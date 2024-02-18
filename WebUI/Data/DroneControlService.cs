using System.Collections.Generic;
using System;
using System.Diagnostics;
using System.Text;
using NetMQ;
using NetMQ.Sockets;

namespace WebUI.Data
{
    public class DroneControlService : IDisposable
    {
        private Thread thread;
        private bool running = true;
        private Queue<String> outQueue;
        private Queue<String> inQueue;

        public DroneControlService()
        {
            outQueue = new Queue<string>();
            inQueue = new Queue<string>();

            string home = System.Environment.GetFolderPath(Environment.SpecialFolder.UserProfile);
            RunCommandWithBash(home + "/code/parrot-olympe/out/olympe-linux/pyenv_root/versions/3.10.8/bin/python3 " + home +"/NOCTIS/DroneControl/dronecontrol.py");

            Thread.Sleep(2000);

            thread = new Thread(() =>
            {
                using (var client = new RequestSocket())
                {
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
                    }
                }

                Console.WriteLine("DCS Thread Exiting.");
            });
            thread.Start();
        }

        public void SayHello()
        {
            lock (outQueue)
            {
                outQueue.Enqueue("Hello!");
            }
        }

        public byte[] GetCurrentFrame()
        {
            lock (outQueue)
            {
                outQueue.Enqueue("GetCurrentFrame");
            }

            string rsp = "";
            while(!CheckResponses(ref rsp));

            return Encoding.GetEncoding("ISO-8859-1").GetBytes(rsp);
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

        public void Dispose()
        {
            running = false;
            thread.Interrupt();

            thread.Join(1000);
        }

        private static void RunCommandWithBash(string command)
        {
            Process.Start("/bin/bash", $"-c \"{command}\"");
        }   
    }
}
