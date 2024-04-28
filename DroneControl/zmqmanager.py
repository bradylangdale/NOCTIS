import zmq
from threading import Thread
from dronehandler import DroneHandler
from geofencemanager import GeofenceManager
import json
import numpy as np
import cv2
import time
from dronehandler import DroneState

JPG_QUALITY = [int(cv2.IMWRITE_JPEG_QUALITY), 30]


class ZMQManager:

    def __init__(self):
        self.running = True
        self.messages = []

        self.drone = DroneHandler(self)
        self.geofencemanager = GeofenceManager(self)

        self.drone.set_geo(self.geofencemanager)

        self.zmq_thread = Thread(target=self._zmqManager)
        self.zmq_thread.run()

    def _zmqManager(self):
        print('Connecting to the Web UI…')
        context = zmq.Context()
        socket = context.socket(zmq.REP)
        socket.bind('tcp://*:5555')

        self.log('Drone Control Service is alive!', level='SUCCESS')

        while(self.running):
            message = socket.recv().decode('utf-8')

            if message == 'Survey':

                socket.send_string('ECHO: Survey')

                self.drone.state = DroneState.TakingOff
                Thread(target=self.drone.fly).start()
                self.log('Starting Automated Survey.')

            elif message == 'GetCurrentFrame':
                socket.send_string(str(self.drone.display_num.value))

            elif message == 'ConnectDrone':

                socket.send_string('ECHO: ConnectDrone')

                Thread(target=self.drone.start).start()

            elif message == 'DisconnectDrone':

                socket.send_string('ECHO: DisconnectDrone')

                Thread(target=self.drone.stop).start()

            elif message[:9] == 'Geofence:':

                # TODO: rename this to SetGeofence
                socket.send_string('ECHO: Geofence')

                Thread(target=self.geofencemanager.update_geo_nav_data, args=(message[10:],)).start()
                self.log('Geofence saving.')

            elif message == 'GetGeofence':

                # This is quick
                socket.send_string('ECHO: GetGeofence')

                try:
                    with open('../DroneControl/data/geofence.json', 'r') as f:
                        self.log(f.read(), 'GEOFENCE')
                except:
                    self.log('[]', 'GEOFENCE')

            elif message[:9] == 'MakePath:':

                socket.send_string('ECHO: MakePath')

                markers = json.loads(message[10:])
                if len(markers) == 2:
                    Thread(target=self.geofencemanager.get_path, args=(markers[0], markers[1], True)).start()
                    self.log('MakePath called!', level='DEBUG')
                else:
                    self.log('[]', level='PATH')
                    self.log('MakePath needs two markers.', level='ERROR')

            elif message[:11] == 'MakeSurvey:':

                socket.send_string('ECHO: MakeSurvey')

                markers = json.loads(message[12:])
                if len(markers) == 1:
                    Thread(target=self.geofencemanager.get_survey_path, args=(markers[0], True)).start()
                    self.log('MakeSurvey called!', level='DEBUG')
                else:
                    self.log('[]', level='SURVEY')
                    self.log('MakeSurvey needs one marker.', level='ERROR')

            elif message == 'CheckLogs':

                if len(self.messages) > 0:
                    socket.send_string(self.messages.pop(0))
                else:
                    socket.send_string('No Messages')

            elif message == "Terminate":

                Thread(target=self.drone.stop).start()
                self.running = False

                time.sleep(0.25)

                socket.send_string('Successful Shutdown Drone Control.')
            elif message == "Thermal":
                
                socket.send_string('ECHO: Thermal')
                self.drone.toggle_thermal()

                self.log('Thermal Settings Set!', level='DEBUG')

                #self.log('Drone Control requires full restart. Restarting now!', level='WARNING')
            
            elif message == "Detection":

                socket.send_string('ECHO: Detection')
                self.drone.toggle_detection()

                self.log('Object Detection Toggled!', level='DEBUG')
                
            else:
                socket.send_string('ECHO: ' + message)
                self.log('Received Unknown Message: ' + message, level='ERROR')

        print('ZMQ has shutdown!')
        time.sleep(0.25)
        socket.close()

    def log(self, msg, level='LOG'):
        self.messages.append(level + ': ' + str(msg))
