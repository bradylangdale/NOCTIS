using System.Collections.Generic;
using System;
using System.Diagnostics;
using System.Text;
using NetMQ;
using NetMQ.Sockets;
using System.Threading.Tasks.Dataflow;
using System.Drawing.Drawing2D;

namespace WebUI.Data
{
    public class DroneControlService : IDisposable
    {
        private Thread? thread;
        private bool running = true;
        private Queue<string> outQueue;
        private Queue<string> inQueue;

        private uint LOG_LENGTH = 120;
        private Queue<string> logs;

        private Queue<string> pathQueue;
        private Queue<string> surveyQueue;

        private bool droneConnected = false;

        private string imagePath = "/icons/stream_default.png";

        public DroneControlService()
        {
            outQueue = new Queue<string>();
            inQueue = new Queue<string>();
            logs = new Queue<string>();
            pathQueue = new Queue<string>();
            surveyQueue = new Queue<string>();

            StartDroneService();
        }

        public void SendCommand(string cmd)
        {
            if (!running) return;

            lock (outQueue)
            {
                outQueue.Enqueue(cmd);
            }
        }

        public void StreamState(bool state)
        {
            droneConnected = state;

            if (!state) imagePath = "/icons/stream_default.png";
        }

        public string GetCurrentFrame()
        {
            lock (imagePath)
            {
                return imagePath;
            }
        }

        public bool CheckResponses(ref string response, bool log = true)
        {
            if (!running) return false;

            lock (inQueue)
            {
                if (inQueue.Count > 0)
                {
                    response = inQueue.Dequeue();
                    if (log) logs.Enqueue(System.DateTime.Now.ToString("MM/dd/yyyy HH:mm:ss: ") + response);
                    return true;
                } else {
                    return false;
                }
            }
        }

        public bool CheckPathResponses(ref string response)
        {
            if (!running) return false;

            while (pathQueue.Count == 0) Thread.Sleep(100);

            lock (pathQueue)
            {
                if (pathQueue.Count > 0)
                {
                    response = pathQueue.Dequeue();
                    logs.Enqueue(System.DateTime.Now.ToString("MM/dd/yyyy HH:mm:ss: ") + "DEBUG: Received test path.");
                    return true;
                } else {
                    return false;
                }
            }
        }

        public bool CheckSurveyResponses(ref string response)
        {
            if (!running) return false;

            while (surveyQueue.Count == 0) Thread.Sleep(100);

            lock (surveyQueue)
            {
                if (surveyQueue.Count > 0)
                {
                    response = surveyQueue.Dequeue();
                    logs.Enqueue(System.DateTime.Now.ToString("MM/dd/yyyy HH:mm:ss: ") + "DEBUG: Received test survey path.");
                    return true;
                } else {
                    return false;
                }
            }
        }

        public bool GetLogs(ref List<string[]> response)
        {
            if (!running) return false;

            response.Clear();

            lock (logs)
            {
                foreach (string line in logs)
                {
                    response.Add(line.Split(": "));
                }
            }

            return true;
        }

        public void RestartDroneService()
        {
            try {
                // for good measure
                RunCommandWithBash("killall -9 python3");

                running = false;
                if (thread is not null) {
                    thread.Join(1000);
                }

                running = true;
                droneConnected = false;

                outQueue = new Queue<string>();
                inQueue = new Queue<string>();
                pathQueue = new Queue<string>();
                surveyQueue = new Queue<string>();

                logs = new Queue<string>();

                StartDroneService();
            } catch (Exception e)
            {
                Console.WriteLine(e);
            }
        }

        public void Dispose()
        {
            running = false;

            if (thread is not null)
            {
                thread.Interrupt();
                thread.Join(250);
            }
        }

        private void StartDroneService()
        {
            string home = System.Environment.GetFolderPath(Environment.SpecialFolder.UserProfile);
            string cwd = System.IO.Directory.GetCurrentDirectory().Replace("WebUI", "");

            thread = new Thread(() =>
            {
                RunCommandWithBash(home + "/code/parrot-olympe/out/olympe-linux/pyenv_root/versions/3.10.8/bin/python3 " + cwd +"/DroneControl/dronecontrol.py");
                Thread.Sleep(250);
                using (var client = new RequestSocket())
                {   
                    int display_num = 0;
                    int write_num = 1;

                    client.Connect("tcp://localhost:5555");
                    while (running)
                    {
                        lock (outQueue)
                        {
                            if (outQueue.Count > 0)
                            {
                                try {
                                    client.SendFrame(outQueue.Dequeue());
                                    
                                    string message = client.ReceiveFrameString();

                                    // ok now do something
                                    lock (inQueue)
                                    {
                                        inQueue.Enqueue(message);
                                    }
                                } catch (Exception e) {
                                    Console.WriteLine(e);
                                }
                            }
                        }

                        lock (logs) {
                            client.SendFrame("CheckLogs");

                            string line = client.ReceiveFrameString();

                            while (line != "No Messages")
                            {
                                if (line.Contains("PATH: "))
                                {
                                    line = line.Replace("PATH: ", "");

                                    lock (pathQueue)
                                    {
                                        pathQueue.Enqueue(line);
                                    }

                                    client.SendFrame("CheckLogs");

                                    line = client.ReceiveFrameString();
                                } else if (line.Contains("SURVEY: "))
                                {
                                    line = line.Replace("SURVEY: ", "");

                                    lock (surveyQueue)
                                    {
                                        surveyQueue.Enqueue(line);
                                    }

                                    client.SendFrame("CheckLogs");

                                    line = client.ReceiveFrameString();
                                } else 
                                {
                                    if (line.Contains("LOG: Connected to the drone."))
                                    {
                                        StreamState(true);
                                    }

                                    if (line.Contains("LOG: Disconnected the drone."))
                                    {
                                        StreamState(false);
                                    }

                                    if (logs.Count > LOG_LENGTH) logs.Dequeue();

                                    logs.Enqueue(System.DateTime.Now.ToString("MM/dd/yyyy HH:mm:ss: ") + line);

                                    client.SendFrame("CheckLogs");

                                    line = client.ReceiveFrameString();
                                }
                            }
                        }

                        try {
                            if (droneConnected)
                            {
                                client.SendFrame("GetCurrentFrame");

                                string str_count = write_num.ToString();
                                str_count = (str_count == "") ? "0" : str_count;

                                string frame = client.ReceiveFrameString();
                                if (frame != "Not Ready")
                                {
                                    System.IO.File.WriteAllBytes(
                                        home + "/Projects/NOCTIS/WebUI/wwwroot/Data/Videos/current_frame" + str_count + ".jpg",
                                        Encoding.GetEncoding("ISO-8859-1").GetBytes(frame)
                                    );

                                    write_num += 1;
                                    write_num %= 32;

                                    str_count = display_num.ToString();

                                    str_count = (str_count == "") ? "0" : str_count;

                                    lock (imagePath)
                                    {
                                        imagePath = "/Data/Videos/current_frame" + str_count + ".jpg" + "?DummyId=" + DateTime.Now.Ticks;
                                    }

                                    display_num += 1;
                                    display_num %= 32;
                                } else {
                                    lock (imagePath)
                                    {
                                        imagePath = "/icons/stream_default.png";
                                    }
                                }
                            }

                        } catch (Exception e) {
                            Console.WriteLine(e);

                            lock (imagePath)
                            {
                                imagePath = "/icons/stream_default.png";
                            }
                        }

                        Thread.Sleep(100);
                    }
                }

                Console.WriteLine("DCS Thread Exiting.");
            });
            thread.Start();
        }

        private static void RunCommandWithBash(string command)
        {
            Process.Start("/bin/bash", $"-c \"{command}\"");
        }   
    }
}
