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
        
        private uint TIMEOUT = 15;

        private uint LOG_LENGTH = 360;
        private Queue<string[]> logs;
        private int log_update = 0;

        private Queue<string> pathQueue;
        private Queue<string> surveyQueue;

        private bool droneConnected = false;

        private string imagePath = "/icons/stream_default.png";
        private string mapData = "";

        // Drone SOH parameters
        private string soh_connection = "Disconnected";
        private string soh_battery = "N/A";
        private string soh_cams = "N/A";
        private string soh_imu = "N/A";
        private string soh_mag = "N/A";
        private string soh_baro = "N/A";
        private string soh_gps = "N/A";
        private string soh_ultra = "N/A";
        private string soh_vert = "N/A";
        private string soh_lat = "N/A";
        private string soh_lng = "N/A";

        public DroneControlService()
        {
            outQueue = new Queue<string>();
            inQueue = new Queue<string>();
            logs = new Queue<string[]>();
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

        public void LogMessage(string msg, string level = "LOG")
        {
            ++log_update;
            logs.Enqueue((System.DateTime.Now.ToString("MM/dd/yyyy HH:mm:ss: ") + level + ": " + msg).Split(": "));
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
                    if (log)
                    {
                        ++log_update;
                        logs.Enqueue((System.DateTime.Now.ToString("MM/dd/yyyy HH:mm:ss: ") + response).Split(": "));
                    }
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
                    
                    ++log_update;
                    logs.Enqueue((System.DateTime.Now.ToString("MM/dd/yyyy HH:mm:ss: ") + "DEBUG: Generated test path.").Split(": "));
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

                    ++log_update;
                    logs.Enqueue((System.DateTime.Now.ToString("MM/dd/yyyy HH:mm:ss: ") + "DEBUG: Generated test survey path.").Split(": "));
                    return true;
                } else {
                    return false;
                }
            }
        }

        public bool CheckGeofenceResponse(ref string response)
        {
            if (!running) return false;

            while (mapData == "") Thread.Sleep(100);

            lock (mapData)
            {
                response = mapData;
                ++log_update;
                logs.Enqueue((System.DateTime.Now.ToString("MM/dd/yyyy HH:mm:ss: ") + "DEBUG: Receive updated geofence.").Split(": "));
                mapData = "";
                return true;
            }
        }

        public int GetLogs(ref List<string[]> response)
        {
            lock (logs)
            {
                response = logs.ToList();
            }

            return log_update;
        }

        public void GetSOH(ref string con, ref string bat, ref string cam,
            ref string imu, ref string mag, ref string baro, ref string gps,
            ref string ultra, ref string vert, ref string lat, ref string lng)
        {
            con = soh_connection;
            bat = soh_battery;
            cam = soh_cams;
            imu = soh_imu;
            mag = soh_mag;
            baro = soh_baro;
            gps = soh_gps;
            ultra = soh_ultra;
            vert = soh_vert;
            lat = soh_lat;
            lng = soh_lng;
        }

        public void GetDronePosition(ref string lat, ref string lng)
        {
            lat = soh_lat;
            lng = soh_lng;
        }

        public void RestartDroneService()
        {
            try {
                // for good measure
                RunCommandWithBash("killall -9 python3 pt_main_thread");

                running = false;
                if (thread is not null) {
                    thread.Join(1000);
                }

                running = true;
                StreamState(false);

                outQueue = new Queue<string>();
                inQueue = new Queue<string>();
                pathQueue = new Queue<string>();
                surveyQueue = new Queue<string>();

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
                RunCommandWithBash(home + "/code/parrot-olympe/out/olympe-linux/pyenv_root/versions/3.11.9/bin/python3 " + cwd +"/DroneControl/dronecontrol.py");
                Thread.Sleep(5000);
                using (var client = new RequestSocket())
                {   
                    string last_num = "-2";

                    client.Connect("tcp://localhost:5555");
                    while (running)
                    {
                        lock (outQueue)
                        {
                            if (outQueue.Count > 0)
                            {
                                try {
                                    client.SendFrame(outQueue.Dequeue());
                                    
                                    string? message = "";
                                    if (!client.TryReceiveFrameString(TimeSpan.FromSeconds(TIMEOUT), out message))
                                    {
                                        LogMessage("Drone Control Service took too long to respond forcing restart.", "ERROR");
                                        StreamState(false);
                                        Thread.Sleep(500);
                                        running = false;
                                        RestartDroneService();
                                        break;
                                    }

                                    // ok now do something
                                    lock (inQueue)
                                    {
                                        inQueue.Enqueue(message);
                                    }

                                    if (message == "Successful Shutdown Drone Control.")
                                    {
                                        StreamState(false);
                                        Thread.Sleep(500);
                                        running = false;
                                        RestartDroneService();
                                        break;
                                    }
                                } catch (Exception e) {
                                    Console.WriteLine(e);
                                }
                            }
                        }

                        lock (logs) {
                            client.SendFrame("CheckLogs");

                            string? line = "";
                            if (!client.TryReceiveFrameString(TimeSpan.FromSeconds(TIMEOUT), out line))
                            {
                                LogMessage("Drone Control Service took too long to respond forcing restart.", "ERROR");
                                StreamState(false);
                                Thread.Sleep(500);
                                running = false;
                                RestartDroneService();
                                break;
                            }

                            while (line != "No Messages" && running)
                            {
                                if (line.Contains("PATH: "))
                                {
                                    line = line.Replace("PATH: ", "");

                                    lock (pathQueue)
                                    {
                                        pathQueue.Enqueue(line);
                                    }

                                    client.SendFrame("CheckLogs");

                                    line = "";
                                    if (!client.TryReceiveFrameString(TimeSpan.FromSeconds(TIMEOUT), out line))
                                    {
                                        LogMessage("Drone Control Service took too long to respond forcing restart.", "ERROR");
                                        StreamState(false);
                                        Thread.Sleep(500);
                                        running = false;
                                        RestartDroneService();
                                        break;
                                    }
                                } else if (line.Contains("SURVEY: "))
                                {
                                    line = line.Replace("SURVEY: ", "");

                                    lock (surveyQueue)
                                    {
                                        surveyQueue.Enqueue(line);
                                    }

                                    client.SendFrame("CheckLogs");

                                    line = "";
                                    if (!client.TryReceiveFrameString(TimeSpan.FromSeconds(TIMEOUT), out line))
                                    {
                                        LogMessage("Drone Control Service took too long to respond forcing restart.", "ERROR");
                                        StreamState(false);
                                        Thread.Sleep(500);
                                        running = false;
                                        RestartDroneService();
                                        break;
                                    }
                                } else if (line.Contains("SOH: "))
                                {
                                    line = line.Replace("SOH: ", "");
                                    
                                    string[] soh = line.Split(",");

                                    soh_connection = soh[0];
                                    soh_battery = soh[1];
                                    soh_cams = soh[2];
                                    soh_imu = soh[3];
                                    soh_mag = soh[4];
                                    soh_baro = soh[5];
                                    soh_gps = soh[6];
                                    soh_ultra = soh[7];
                                    soh_vert = soh[8];
                                    soh_lat = soh[9];
                                    soh_lng = soh[10];

                                    client.SendFrame("CheckLogs");

                                    line = "";
                                    if (!client.TryReceiveFrameString(TimeSpan.FromSeconds(TIMEOUT), out line))
                                    {
                                        LogMessage("Drone Control Service took too long to respond forcing restart.", "ERROR");
                                        StreamState(false);
                                        Thread.Sleep(500);
                                        running = false;
                                        RestartDroneService();
                                        break;
                                    }
                                } else if (line.Contains("GEOFENCE: "))
                                {
                                    line = line.Replace("GEOFENCE: ", "");

                                    lock (mapData)
                                    {
                                        mapData = line;
                                    }

                                    client.SendFrame("CheckLogs");

                                    line = "";
                                    if (!client.TryReceiveFrameString(TimeSpan.FromSeconds(TIMEOUT), out line))
                                    {
                                        LogMessage("Drone Control Service took too long to respond forcing restart.", "ERROR");
                                        StreamState(false);
                                        Thread.Sleep(500);
                                        running = false;
                                        RestartDroneService();
                                        break;
                                    }
                                } else 
                                {
                                    if (line.Contains("LOG: Connected to the Skycontroller."))
                                    {
                                        StreamState(true);
                                    }

                                    if (line.Contains("LOG: Disconnected from the Skycontroller."))
                                    {
                                        StreamState(false);
                                    }

                                    if (logs.Count > LOG_LENGTH) logs.Dequeue();

                                    ++log_update;
                                    logs.Enqueue((System.DateTime.Now.ToString("MM/dd/yyyy HH:mm:ss: ") + line).Split(": "));

                                    if (line.Contains("Drone Control requires full restart. Restarting now!"))
                                    {
                                        LogMessage("Drone Control Service took too long to respond forcing restart.", "ERROR");
                                        StreamState(false);
                                        Thread.Sleep(500);
                                        running = false;
                                        RestartDroneService();
                                        break;
                                    }

                                    client.SendFrame("CheckLogs");

                                    line = "";
                                    if (!client.TryReceiveFrameString(TimeSpan.FromSeconds(TIMEOUT), out line))
                                    {
                                        LogMessage("Drone Control Service took too long to respond forcing restart.", "ERROR");
                                        StreamState(false);
                                        Thread.Sleep(500);
                                        running = false;
                                        RestartDroneService();
                                        break;
                                    }
                                }
                            }
                        }

                        try {
                            if (droneConnected)
                            {
                                client.SendFrame("GetCurrentFrame");

                                string? frame = "";
                                if (!client.TryReceiveFrameString(TimeSpan.FromSeconds(TIMEOUT), out frame))
                                {
                                    LogMessage("Drone Control Service took too long to respond forcing restart.", "ERROR");
                                    StreamState(false);
                                    Thread.Sleep(500);
                                    running = false;
                                    RestartDroneService();
                                    break;
                                }

                                if (frame != "-1")
                                {
                                    if (frame != last_num)
                                    {
                                        last_num = frame;

                                        lock (imagePath)
                                        {
                                            imagePath = "/Data/Videos/current_frame" + frame + ".jpg" + "?DummyId=" + DateTime.Now.Ticks;
                                        }
                                    }
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

        public void RunCommandWithBash(string command)
        {
            Process.Start("/bin/bash", $"-c \"{command}\"");
        }   
    }
}
