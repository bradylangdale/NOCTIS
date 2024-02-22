from threading import Thread
import time

import math
import os
import subprocess
import cv2
import numpy as np
from enum import Enum

import olympe
from olympe.messages.ardrone3.Piloting import TakeOff, Landing
from olympe.messages.ardrone3.Piloting import moveBy
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged, AltitudeAboveGroundChanged
from olympe.messages.ardrone3.PilotingSettings import MaxTilt
from olympe.messages.ardrone3.PilotingSettingsState import MaxTiltChanged
from olympe.messages.ardrone3.GPSSettingsState import GPSFixStateChanged
from olympe.messages.common.CommonState import BatteryStateChanged, SensorsStatesListChanged
from olympe.messages.camera import camera_states
from olympe.enums.common.CommonState import SensorsStatesListChanged_SensorName as Sensor


olympe.log.update_config({"loggers": {"olympe": {"level": "WARNING"}}})

DRONE_IP = os.environ.get("DRONE_IP", "10.202.0.1")
DRONE_RTSP_PORT = os.environ.get("DRONE_RTSP_PORT")


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

    def __init__(self):
        super().__init__()

        # Create the olympe.Drone object from its IP address
        self.drone = olympe.Drone(DRONE_IP)
        
        subprocess.run(f'mkdir -p {os.getcwd()}/wwwroot/Data/'.split(' '))
        subprocess.run(f'mkdir -p {os.getcwd()}/wwwroot/Data/Videos/'.split(' '))

        self.video_thread = None
        self.running = False
        self.current_frame = None
        self.survey_complete = True
        self.state = DroneState.Idling

    def start(self):
        # Connect to drone
        assert self.drone.connect(retry=3)
        self.running = True

        self.video_thread = Thread(target=self.frame_processing)
        self.video_thread.start()

    def stop(self):
        assert self.drone.disconnect()
        self.running = False
        
        time.sleep(10)
        self.video_thread.join()


    def frame_processing(self):

        # TODO: check if I can just output jpeg instead raw
        command = [
            'ffmpeg',
            '-probesize', '32',
            '-analyzeduration', '0',
            '-loglevel', 'quiet',
            '-i', f'rtsp://{DRONE_IP}/live',
            '-f', 'rawvideo',
            '-pix_fmt', 'bgr24',
            'pipe:'
        ]

        # TODO: fix this so when the application is closed stdout is reclaimed
        p = subprocess.Popen(command, stdout=subprocess.PIPE)

        width = 1280
        height = 720

        while self.running:

            try:
                raw_frame = p.stdout.read(width*height*3)
                frame = np.frombuffer(raw_frame, np.uint8)
                frame = frame.reshape((height, width, 3))

                self.current_frame = cv2.imencode(".jpg", frame)[1].tobytes()
                
            except Exception as e:
                p.kill()
                time.sleep(500)
                p = subprocess.Popen(command, stdout=subprocess.PIPE)
                print(e)

        p.kill()

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

    def takingoff(self):
        # check battery level
        battery_level = self.drone.get_state(BatteryStateChanged)["percent"]
        if battery_level < 90:
            self.state = DroneState.Charging
            return

        # check cameras running
        if self.drone.get_state(camera_states)["active_cameras"] == 0:
            self.state = DroneState.Idling
            return

        # check drone sensors
        sensors = self.drone.get_state(SensorsStatesListChanged)

        if sensors[Sensor.IMU]['sensorState'] == 0:
            self.state = DroneState.Idling
            return

        if sensors[Sensor.magnetometer]['sensorState'] == 0:
            self.state = DroneState.Idling
            return
        
        if sensors[Sensor.barometer]['sensorState'] == 0:
            self.state = DroneState.Idling
            return
        
        if sensors[Sensor.GPS]['sensorState'] == 0:
            self.state = DroneState.Idling
            return
        
        if sensors[Sensor.ultrasound]['sensorState'] == 0:
            self.state = DroneState.Idling
            return
        
        if sensors[Sensor.vertical_camera]['sensorState'] == 0:
            self.state = DroneState.Idling
            return
        
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

        # TODO: determine if we need to land or go into idle
        self.state = DroneState.Surveying

    def surveying(self):
        maxtilt = self.drone.get_state(MaxTiltChanged)["max"]
        self.drone(MaxTilt(maxtilt)).wait()

        self.drone(moveBy(-100, 0, -25, 0, _timeout=20)).wait().success()

        for i in range(4):
            self.drone(moveBy(50, 0, 0, math.pi, _timeout=20)).wait().success()

        self.state = DroneState.Landing

    def landing(self):
        self.drone(Landing() >> FlyingStateChanged(state="landed", _timeout=5)).wait()

        self.state = DroneState.Idling


