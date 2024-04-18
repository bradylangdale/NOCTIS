from threading import Thread
import time

import math
import os
import subprocess
import cv2
import numpy as np
from enum import Enum

os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "rtsp_transport;udp"

import olympe
import olympe_deps as od
from olympe.messages.skyctrl.CoPiloting import setPilotingSource
from olympe.messages.ardrone3.Piloting import TakeOff, Landing
from olympe.messages.ardrone3.Piloting import moveBy, moveTo
from olympe.messages.ardrone3.PilotingState import (FlyingStateChanged, 
                                                    AltitudeAboveGroundChanged, 
                                                    PositionChanged, 
                                                    SpeedChanged,
                                                    moveToChanged)
from olympe.messages.ardrone3.PilotingSettings import MaxTilt
from olympe.messages.ardrone3.PilotingSettingsState import MaxTiltChanged
from olympe.messages.ardrone3.GPSSettingsState import GPSFixStateChanged
from olympe.messages.common.CommonState import BatteryStateChanged, SensorsStatesListChanged
from olympe.messages.camera import camera_states

from olympe.enums.common.CommonState import SensorsStatesListChanged_SensorName as Sensor
from olympe.enums.ardrone3.Piloting import MoveTo_Orientation_mode
from multiprocessing import Process, Manager
from ctypes import c_bool


olympe.log.update_config({"loggers": {"olympe": {"level": "WARNING"}}})

SIM = False

if SIM:
    DRONE_IP = os.environ.get("DRONE_IP", "10.202.0.1")
else:
    DRONE_IP = os.environ.get("DRONE_IP", "192.168.53.1")

DRONE_RTSP_PORT = os.environ.get("DRONE_RTSP_PORT", '554')


class DroneState(Enum):
    Idling = 0
    TakingOff = 1
    Surveying = 2
    Chasing = 3
    Returning = 4
    Landing = 5
    Repositioning = 6
    Charging = 7


class DroneHandler(olympe.EventListener):

    def __init__(self, zmqmanager):
        super().__init__()

        # Create the olympe.Drone object from its IP address
        if SIM:
            self.drone = olympe.Drone(DRONE_IP)
        else:
            self.drone = olympe.Drone(DRONE_IP, drone_type=od.ARSDK_DEVICE_TYPE_ANAFI_THERMAL)

        subprocess.run(f'mkdir -p {os.getcwd()}/wwwroot/Data/'.split(' '))
        subprocess.run(f'mkdir -p {os.getcwd()}/wwwroot/Data/Videos/'.split(' '))

        self.manager = Manager()
        self.shared_frames = self.manager.list()

        self.video_thread = None
        self.running = self.manager.Value(c_bool, False)
        self.survey_complete = True
        self.geofencemanager = None
        self.zmqmanager = zmqmanager
        self.state = DroneState.Idling

        self.start_lat = 0.0
        self.start_lng = 0.0
        self.start_alt = 0.0

        self.path_complete = False
        self.waypoints = []
        self.current_i = 0

        self.last_connection_state = False
        Thread(target=self.heartbeat).start()

    def start(self):
        if self.running.value:
            self.zmqmanager.log('Already connected.', level='DEBUG')
            return True

        # Connect to drone
        if self.drone.connect(retry=3):
            self.drone(setPilotingSource(source="Controller")).wait()
            self.running.value = True
            self.video_thread = Process(target=self.frame_processing)
            self.video_thread.start()
            
            self.zmqmanager.log('Connected to the drone.')
            return True
        
        self.zmqmanager.log('Failed to connect to the drone.', level='ERROR')
        return False

    def stop(self): 
        if not self.running.value:
            self.zmqmanager.log('Already disconnected.', level='DEBUG')
            return True
        
        if self.drone.disconnect(timeout=1):
            self.zmqmanager.log('Disconnected the drone.')

            if self.running.value:
                self.running.value = False
                self.shared_frames[:] = []

                time.sleep(100)

            return True

        self.zmqmanager.log('Failed to disconnect was a drone connected?', level='ERROR')
        return False
    
    def heartbeat(self):
        while self.zmqmanager.running:       
            if self.drone.connection_state() != self.last_connection_state:
                self.last_connection_state = self.drone.connection_state()
                if self.last_connection_state:
                    self.zmqmanager.log('Drone connection successful!')
                else:
                    self.zmqmanager.log('Drone connection failed!')
                    Thread(target=self.stop).start()

            if not self.drone.connection_state():
                self.zmqmanager.log('Attempting to reconnect!')
                self.start()

            time.sleep(1)

    def set_geo(self, geo):
        self.geofencemanager = geo

    def frame_processing(self):

        WEBCAM_RAW_RES = (1280, 720)
        FRAMERATE = 24
        vcap = cv2.VideoCapture(f'rtsp://{DRONE_IP}:{DRONE_RTSP_PORT}/live')
        vcap.set(cv2.CAP_PROP_FPS, FRAMERATE)
        vcap.set(cv2.CAP_PROP_FRAME_WIDTH, WEBCAM_RAW_RES[0])
        vcap.set(cv2.CAP_PROP_FRAME_HEIGHT, WEBCAM_RAW_RES[1])
        vcap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))

        # capture every third frame
        count = 0
        while self.running.value:
            
            if count == 2:
                try:
                    ret, frame = vcap.read()
                    if ret == False:
                        pass
                    else:
                        if len(self.shared_frames) > 0:
                            self.shared_frames[0] = frame
                        else:
                            self.shared_frames.append(frame)
                except Exception as e:
                    print(e)

                count = 0
            else:
                vcap.grab()
                count += 1

    #@olympe.listen_event(AltitudeAboveGroundChanged(_policy='wait'))
    #def onAltitudeAboveGroundChanged(self, event, scheduler):
    #    # TODO: adjust next target to ensure that the drone isn't too close to the ground
    #    pass

    def fly(self):
        self.survey_complete = False
        self.state = DroneState.TakingOff

        while not self.survey_complete:
            if self.state == DroneState.Idling:
                self.survey_complete = True
            elif self.state == DroneState.TakingOff:
                self.takingoff()
            elif self.state == DroneState.Surveying:
                self.surveying()
            elif self.state == DroneState.Chasing:
                self.chasing()
            elif self.state == DroneState.Returning:
                self.returning()
            elif self.state == DroneState.Landing:
                self.landing()
            elif self.state == DroneState.Repositioning:
                self.repositioning()
            elif self.state == DroneState.Charging:
                self.charging()

    def charging(self):
        self.state = DroneState.Idling

    def takingoff(self):
        # check battery level
        battery_level = self.drone.get_state(BatteryStateChanged)["percent"]
        if battery_level < 90:
            self.state = DroneState.Charging
            self.zmqmanager.log('Drone needs to charge.', level='WARNING')
            return

        # check cameras running
        if self.drone.get_state(camera_states)["active_cameras"] == 0:
            self.state = DroneState.Idling
            self.zmqmanager.log('Drone cameras are not functional.', level='ERROR')
            return

        # check drone sensors
        sensors = self.drone.get_state(SensorsStatesListChanged)

        if sensors[Sensor.IMU]['sensorState'] == 0:
            self.state = DroneState.Idling
            self.zmqmanager.log('Drone IMU is not functional.', level='ERROR')
            return

        if sensors[Sensor.magnetometer]['sensorState'] == 0:
            self.state = DroneState.Idling
            self.zmqmanager.log('Drone magnetometer is not functional.', level='ERROR')
            return

        if sensors[Sensor.barometer]['sensorState'] == 0:
            self.state = DroneState.Idling
            self.zmqmanager.log('Drone barometer is not functional.', level='ERROR')
            return

        if sensors[Sensor.GPS]['sensorState'] == 0:
            self.state = DroneState.Idling
            self.zmqmanager.log('Drone GPS is not functional.', level='ERROR')
            return

        if sensors[Sensor.ultrasound]['sensorState'] == 0:
            self.state = DroneState.Idling
            self.zmqmanager.log('Drone ultrasound is not functional.', level='ERROR')
            return

        if sensors[Sensor.vertical_camera]['sensorState'] == 0:
            self.state = DroneState.Idling
            self.zmqmanager.log('Drone vertical camera is not functional.', level='ERROR')
            return

        self.zmqmanager.log('Drone is taking off.')

        self.drone(
            FlyingStateChanged(state="hovering", _policy="check")
            | FlyingStateChanged(state="flying", _policy="check")
            | (
                GPSFixStateChanged(fixed=1, _timeout=10, _policy="check_wait")
                >> (
                    TakeOff(_no_expect=True)
                    & FlyingStateChanged(
                        state="hovering", _timeout=10, _policy="check_wait"
                    )
                )
            )
        ).wait()

        position = self.drone.get_state(PositionChanged)
        self.start_lat = position['latitude']
        self.start_lng = position['longitude']
        self.start_alt = position['altitude']

        self.drone(moveTo(self.start_lat, self.start_lng, 10, MoveTo_Orientation_mode.TO_TARGET, 0, _timeout=1000)).wait().success()

        self.zmqmanager.log('Drone take off is successful going to survey.')

        # TODO: determine if we need to land or go into idle if there's an error
        self.state = DroneState.Surveying

    def surveying(self):
        maxtilt = self.drone.get_state(MaxTiltChanged)["max"]
        self.drone(MaxTilt(maxtilt)).wait()

        self.zmqmanager.log('Drone is generating a surveillance path.')
        survey_path = self.geofencemanager.get_survey_path([self.start_lat, self.start_lng])
        self.zmqmanager.log('Drone has a survey path.')

        #self.waypoints = survey_path

        #self.path_complete = False
        #while not self.path_complete:
        #    time.sleep(1)

        for p in survey_path:
            self.drone(
                moveTo(p[0], p[1], 10, MoveTo_Orientation_mode.TO_TARGET, 0)
                >> moveToChanged(status='DONE', _timeout=500, _float_tol=(1e-05, 1e-06))
                ).wait().success()

        self.zmqmanager.log('Drone has completed surveillance, no targets found.')

        self.state = DroneState.Returning

    def returning(self):
        position = self.drone.get_state(PositionChanged)
        return_path = self.geofencemanager.get_path([position['latitude'], position['longitude']],
                                                    [self.start_lat, self.start_lng])

        self.zmqmanager.log('Drone has generated a return home path.')

        for p in return_path:
            self.drone(
                moveTo(p[0], p[1], 10, MoveTo_Orientation_mode.TO_TARGET, 0)
                >> moveToChanged(status='DONE', _timeout=500, _float_tol=(1e-05, 1e-06))
                ).wait().success()

        self.zmqmanager.log('Drone has made it home. Landing!')

        self.state = DroneState.Landing

    def landing(self):
        self.drone(Landing() >> FlyingStateChanged(state="landed", _timeout=120)).wait().success()

        self.zmqmanager.log('Drone has successfully landed.')

        self.state = DroneState.Idling
