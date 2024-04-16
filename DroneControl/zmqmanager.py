import zmq
from threading import Thread
from dronehandler import DroneHandler
from geofencemanager import GeofenceManager
import json
import numpy as np
import cv2

JPG_QUALITY = [int(cv2.IMWRITE_JPEG_QUALITY), 20]


class ZMQManager:

    def __init__(self):
        self.running = True

        self.drone = DroneHandler()
        self.geofencemanager = GeofenceManager()

        self.drone.set_geo(self.geofencemanager)

        self.zmq_thread = Thread(target=self._zmqManager)
        self.zmq_thread.run()

    def _zmqManager(self):
        print("Connecting to the Web UI…")
        context = zmq.Context()
        socket = context.socket(zmq.REP)
        socket.bind("tcp://*:5555")

        while(self.running):
            message = socket.recv().decode("utf-8")

            if message == 'Survey':
                if self.drone.stop():
                    self.drone.start()
                    t = Thread(target=self.drone.fly)
                    t.start()
                    socket.send_string('Starting Automated Survey.')
                else:
                    socket.send_string('Failed to disconnect was a drone connected?')
            elif message == 'GetCurrentFrame':
                if len(self.drone.frames) > 0:
                    socket.send_string(cv2.imencode(".jpg", self.drone.frames.pop(0), JPG_QUALITY)[1].tobytes().decode('iso-8859-1'))
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
                self.geofencemanager.update_geo_nav_data(message[10:])
                socket.send_string("Geofence Saved!")
            elif message == "GetGeofence":
                try:
                    with open('../DroneControl/data/geofence.json', 'r') as f:
                        socket.send_string(f.read())
                except:
                    socket.send_string('[]')
            elif message[:9] == "MakePath:":
                markers = json.loads(message[10:])
                if len(markers) == 2:
                    socket.send_string(str(self.geofencemanager.get_path(markers[0], markers[1])))
                else:
                    socket.send_string('[]')
            elif message[:11] == "MakeSurvey:":
                markers = json.loads(message[12:])
                if len(markers) == 2:
                    socket.send_string(str(self.geofencemanager.get_survey_path(markers[0])))
                else:
                    socket.send_string('[]')
            else:
                socket.send_string('Unknown Message!')

        socket.close()
