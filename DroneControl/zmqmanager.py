import zmq
from threading import Thread
from dronehandler import DroneHandler
import json


class ZMQManager:

    def __init__(self):
        self.running = True

        self.drone = DroneHandler()

        self.zmq_thread = Thread(target=self._zmqManager)
        self.zmq_thread.run()

    def _zmqManager(self):
        print("Connecting to the Web UI…")
        context = zmq.Context()
        socket = context.socket(zmq.REP)
        socket.bind("tcp://*:5555")

        while(self.running):
            message = socket.recv().decode("utf-8")

            if message == 'TestFlight':
                if self.drone.stop():
                    if self.drone.start():
                        t = Thread(target=self.drone.fly)
                        t.start()
                        socket.send_string('Starting Test Flight.')
                    else:
                        socket.send_string('Failed to connect to the drone.')
                else:
                    socket.send_string('Failed to disconnect was a drone connected?')
            elif message == 'GetCurrentFrame':
                if self.drone.current_frame != None:
                    socket.send_string(self.drone.current_frame.decode('iso-8859-1'))
                else:
                    socket.send_string('Not Ready')
            elif message == 'ConnectDrone':
                if self.drone.start():
                    socket.send_string('Connected to the drone.')
                else:
                    socket.send_string('Failed to connect to the drone.')
            elif message == 'DisconnectDrone':
                if self.drone.stop():
                    socket.send_string('Disconnected the drone.')
                else:
                    socket.send_string('Failed to disconnect was a drone connected?')
            elif message[:9] == "Geofence:":
                socket.send_string("Geofence Saved!")
                print(message[10:])
                res = json.loads(message[10:])
                with open('../DroneControl/data/geofence.json', 'w') as f:
                    json.dump(res, f)
            elif message == "GetGeofence":
                try:
                    with open('../DroneControl/data/geofence.json', 'r') as f:
                        socket.send_string(f.read())
                except:
                    socket.send_string('[]')
            else:
                socket.send_string('Unknown Message!')

        socket.close()
