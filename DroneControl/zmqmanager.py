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

        while(self.running):
            message = socket.recv().decode('utf-8')

            if message == 'Survey':

                socket.send_string('Received: Survey')

                self.log('Stopping Olympe Drone connection.')
                if self.drone.stop():
                    self.log('Olympe Drone connection restarting.')
                    self.drone.start()

                    # TODO: improve this, right now this isn't guarenteed
                    self.log('Olympe Drone connection reestablished.')
                    self.log('This message is not reliable. Verify that the live feed is active.', level='WARNING')

                    Thread(target=self.drone.fly).start()
                    self.log('Starting Automated Survey.')
                else:
                    self.log('Failed to disconnect was a drone connected?', level='ERROR')

            elif message == 'GetCurrentFrame':
                
                # This call is instanteous and always will be?
                #socket.send_string('Received: GetCurrentFrame')

                if len(self.drone.frames) > 0:
                    socket.send_string(cv2.imencode('.jpg', self.drone.frames.pop(0), JPG_QUALITY)[1].tobytes().decode('iso-8859-1'))
                else:
                    socket.send_string('Not Ready')

            elif message == 'ConnectDrone':

                socket.send_string('Received: ConnectDrone')

                if self.drone.start():
                    self.log('Connected to the drone.')
                else:
                    self.log('Failed to connect to the drone.', level='ERROR')

            elif message == 'DisconnectDrone':

                socket.send_string('Received: DisconnectDrone')

                # TODO: is this necessary? it seems duplicated
                if self.drone.stop():
                    self.log('Disconnected the drone.')
                else:
                    self.log('Failed to disconnect was a drone connected?', level='ERROR')

            elif message[:9] == 'Geofence:':

                # TODO: rename this to SetGeofence
                socket.send_string('Received: Geofence')

                Thread(target=self.geofencemanager.update_geo_nav_data, args=(message[10:])).start()
                self.log('Geofence saving.')

            elif message == 'GetGeofence':

                # This is quick
                #socket.send_string('Received: GetGeofence')

                try:
                    with open('../DroneControl/data/geofence.json', 'r') as f:
                        socket.send_string(f.read())
                except:
                    socket.send_string('[]')

            elif message[:9] == 'MakePath:':

                socket.send_string('Received: MakePath')

                markers = json.loads(message[10:])
                if len(markers) == 2:
                    Thread(target=self.geofencemanager.get_path, args=(markers[0], markers[1], True)).start()
                    self.log('MakePath called!', level='DEBUG')
                else:
                    self.log('[]', level='PATH')
                    self.log('MakePath needs two markers.', level='ERROR')

            elif message[:11] == 'MakeSurvey:':

                socket.send_string('Received: MakeSurvey')

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

            else:
                socket.send_string('Received: ' + message)
                self.log('Received Unknown Message: ' + message, level='ERROR')

        socket.close()

    def log(self, msg, level='LOG'):
        self.messages.append(level + ': ' + msg)