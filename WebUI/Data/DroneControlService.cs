using System;
using System.Threading;
using System.Diagnostics;
using NetMQ;
using NetMQ.Sockets;

namespace WebUI.Data
{
    public class DroneControlService
    {
        public DroneControlService()
        {
            var thread = new Thread(() =>
            {
                using (var server = new ResponseSocket("@tcp://*:5555"))
                {
                    while (true)
                    {
                        var message = server.ReceiveFrameString();
                        Console.WriteLine("Received: " + message);
                        server.SendFrame("World");
                    }
                }
            });
            thread.Start();

            RunCommandWithBash("/bin/python3 /home/brady/Projects/NOCTIS/DroneControl/dronecontrol.py");
        }

        private static void RunCommandWithBash(string command)
        {
            Process.Start("/bin/bash", $"-c \"{command}\"");

            //var output = process.StandardOutput.ReadToEnd();

            //Console.WriteLine(output);
        }   
    }
}
