using System.Collections.Generic;
using System;
using System.Diagnostics;
using System.Text;
using NetMQ;
using NetMQ.Sockets;
using System.Threading.Tasks.Dataflow;

namespace WebUI.Data
{
    public class DroneControlService : IDisposable
    {
        private Thread? thread;
        private bool running = true;
        private Queue<String> outQueue;
        private Queue<String> inQueue;

        private bool droneConnected = false;

        private string imagePath = "/icons/stream_default.png";

        public DroneControlService()
        {
            outQueue = new Queue<string>();
            inQueue = new Queue<string>();

            StartDroneService();
        }

        public void SendCommand(string cmd)
        {
            lock (outQueue)
            {
                outQueue.Enqueue(cmd);
            }
        }

        public void StreamState(bool state)
        {
            droneConnected = state;
        }

        public string GetCurrentFrame()
        {
            lock (imagePath)
            {
                return imagePath;
            }
        }

        public bool CheckResponses(ref string response)
        {
            lock (inQueue)
            {
                if (inQueue.Count > 0)
                {
                    response = inQueue.Dequeue();
                    return true;
                } else {
                    return false;
                }
            }
        }

        // TODO: add null checks on thread
        public void RestartDroneService()
        {
            running = false;
            thread.Interrupt();

            thread.Join(1000);

            running = true;
            StartDroneService();
        }

        public void Dispose()
        {
            running = false;
            thread.Interrupt();

            thread.Join(1000);
        }

        private void StartDroneService()
        {
            string home = System.Environment.GetFolderPath(Environment.SpecialFolder.UserProfile);
            string cwd = System.IO.Directory.GetCurrentDirectory().Replace("WebUI", "");
            
            RunCommandWithBash(home + "/code/parrot-olympe/out/olympe-linux/pyenv_root/versions/3.10.8/bin/python3 " + cwd +"/DroneControl/dronecontrol.py");

            Thread.Sleep(2000);

            thread = new Thread(() =>
            {
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

                            try {
                                if (droneConnected)
                                {
                                    client.SendFrame("GetCurrentFrame");

                                    string str_count = write_num.ToString();
                                    str_count = (str_count == "") ? "0" : str_count; 

                                    System.IO.File.WriteAllBytes(
                                        home + "/Projects/NOCTIS/WebUI/wwwroot/Data/Videos/current_frame" + str_count + ".jpg",
                                        Encoding.GetEncoding("ISO-8859-1").GetBytes(client.ReceiveFrameString())
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
                                }

                            } catch (Exception e) {
                                Console.WriteLine(e);

                                lock (imagePath)
                                {
                                    imagePath = "/icons/stream_default.png";
                                }
                            }
                        }

                        Thread.Sleep(64);
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
