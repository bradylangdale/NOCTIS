from threading import Thread
import time

import math
import os
import subprocess
import cv2
import numpy as np
from enum import IntEnum

os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "rtsp_transport;udp"

import olympe
import olympe_deps as od
from olympe.messages.skyctrl.CoPiloting import setPilotingSource
from olympe.messages.ardrone3.Piloting import TakeOff, Landing
from olympe.messages.ardrone3.Piloting import moveBy, moveTo, CancelMoveTo
from olympe.messages.ardrone3.PilotingState import (FlyingStateChanged, 
                                                    AltitudeAboveGroundChanged, 
                                                    PositionChanged,
                                                    SpeedChanged,
                                                    moveToChanged)
from olympe.messages.ardrone3.PilotingSettings import MaxTilt
from olympe.messages.ardrone3.PilotingSettingsState import MaxTiltChanged
from olympe.messages.ardrone3.GPSSettingsState import GPSFixStateChanged, HomeChanged
from olympe.messages.common.CommonState import BatteryStateChanged, SensorsStatesListChanged
from olympe.messages.camera import camera_states
from olympe.messages.gimbal import attitude

from olympe.enums.common.CommonState import SensorsStatesListChanged_SensorName as Sensor
from olympe.enums.ardrone3.Piloting import MoveTo_Orientation_mode
from olympe.messages.thermal import set_mode, set_rendering, set_palette_settings, mode, rendering

from multiprocessing import Process, Manager
from ctypes import c_bool, c_uint16

import json


olympe.log.update_config({"loggers": {"olympe": {"level": "WARNING"}}})

SIM = True

if SIM:
    DRONE_IP = os.environ.get("DRONE_IP", "10.202.0.1")
else:
    DRONE_IP = os.environ.get("DRONE_IP", "192.168.53.1")

DRONE_RTSP_PORT = os.environ.get("DRONE_RTSP_PORT", '554')


class DroneState(IntEnum):
    Idling = 0
    TakingOff = 1
    Surveying = 2
    Chasing = 3
    Returning = 4
    Landing = 5
    Repositioning = 6
    Charging = 7
    Error = 8


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

        self.nonvolatile = { 'state': int(DroneState.Idling) }

        self.manager = Manager()
        self.shared_frames = self.manager.list()

        self.need_video_restart = False
        self.video_thread = None

        self.running = self.manager.Value(c_bool, False)
        self.object_detect = self.manager.Value(c_bool, False)
        self.object_detected = self.manager.Value(c_bool, False)
        self.object_px = self.manager.Value(c_uint16, 0)
        self.object_py = self.manager.Value(c_uint16, 0)

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

        self.thermal_state = None

        self.last_connection_state = False

        try:
            with open('../DroneControl/data/dcs.json', 'r') as f:
                self.nonvolatile = json.load(f)

            self.state = DroneState(self.nonvolatile['state'])
            self.start_lat = self.nonvolatile['home'][0]
            self.start_lng = self.nonvolatile['home'][1]
            
            self.zmqmanager.log('Successfully loaded dcs.json', level='SUCCESS')
        except:
            self.zmqmanager.log('No dcs.json found creating one.', level='WARNING')

            with open('../DroneControl/data/dcs.json', 'w') as f:
                json.dump(self.nonvolatile, f)

        if not(self.state == DroneState.Idling or self.state == DroneState.Charging):
            self.state = DroneState.Error

        Thread(target=self.heartbeat).start()

    def start(self, reattempt=False):
        if self.running.value:
            self.zmqmanager.log('Already connected.', level='WARNING')

            if reattempt:
                self.zmqmanager.log('Skycontroller has not reconnected with the drone, retrying 10 seconds.', level='ERROR')
                self.need_video_restart = True
                time.sleep(9)

            return True

        # Connect to drone
        if self.drone.connect(retry=3):
            self.drone(setPilotingSource(source="Controller")).wait()
            self.running.value = True
            self.video_thread = Process(target=self.frame_processing)
            self.video_thread.start()
            
            self.zmqmanager.log('Connected to the Skycontroller.')
            return True
        
        self.zmqmanager.log('Failed to connect to the drone.', level='ERROR')
        return False

    def stop(self): 
        if not self.running.value:
            self.zmqmanager.log('Already disconnected.', level='WARNING')
            return True
        
        if self.drone.disconnect(timeout=1):
            self.zmqmanager.log('Disconnected from the Skycontroller.')

            if self.running.value:
                self.running.value = False
                self.shared_frames[:] = []

                time.sleep(100)

            return True

        self.zmqmanager.log('Failed to disconnect was a drone connected?', level='ERROR')
        return False
    
    def heartbeat(self):
        last_bat = 0
        cams = "Healthy"
        imu = "Healthy"
        mag = "Healthy"
        baro = "Healthy"
        gps = "Healthy"
        ultra = "Healthy"
        vert = "Healthy"
        lat = "0"
        lng = "0"

        while self.zmqmanager.running:       
            if self.drone.connection_state() != self.last_connection_state:
                self.last_connection_state = self.drone.connection_state()
                if self.last_connection_state:
                    self.zmqmanager.log('Drone connection successful!', level='SUCCESS')

                    if self.need_video_restart:
                        self.zmqmanager.log('Drone Control requires full restart. Restarting now!', level='WARNING')

                        self.need_video_restart = False
                else:
                    self.zmqmanager.log('Drone connection failed!', level='ERROR')
                    self.running.value = False
                    self.shared_frames[:] = []
                    Thread(target=self.stop).start()

            if not self.drone.connection_state():
                time.sleep(0.5)
                self.zmqmanager.log('Attempting to reconnect!', level='WARNING')
                self.start(reattempt=True)

            if self.drone.connection_state():
                # check battery level
                battery_level = self.drone.get_state(BatteryStateChanged)["percent"]

                if battery_level % 25 == 0 or battery_level != last_bat:
                    if battery_level % 25 == 0 and battery_level != last_bat:
                        self.zmqmanager.log('Battery Level at ' + str(battery_level) + '%', level='WARNING')

                    if battery_level != last_bat and battery_level <= 25:
                        if self.state == DroneState.Idling and self.state != DroneState.Charging:
                            self.state = DroneState.Charging
                            Thread(target=self.fly).start()
                            self.zmqmanager.log('Starting drone charging.', level='LOG')
                        elif self.state == DroneState.Surveying or self.state == DroneState.Chasing:
                            self.zmqmanager.log('Battery Level is too low canceling current flight.', level='WARNING')
                            self.state = DroneState.Error
                            self.zmqmanager.log('Drone Control requires full restart. Restarting now!', level='WARNING')
                
                    last_bat = battery_level
                
                if self.state == DroneState.Error:
                    self.zmqmanager.log('Drone Control reset during flight, forcing drone to return home!', level='ERROR')
                    self.state = DroneState.Returning
                    Thread(target=self.fly).start()

                self.nonvolatile['state'] = int(self.state)

                with open('../DroneControl/data/dcs.json', 'w') as f:
                    json.dump(self.nonvolatile, f)

                if not SIM:
                    if str(self.drone.get_state(mode)["mode"]) == "mode.disabled" or \
                    str(self.drone.get_state(rendering)["mode"]) == "rendering_mode.visible":
                        if self.thermal_state != False:
                            self.zmqmanager.log('Thermal Imaging is disabled.')
                            self.thermal_state = False
                            self.object_detect.value = False
                    else:
                        if self.thermal_state != True:
                            self.zmqmanager.log('Thermal Imaging is enabled.')
                            self.thermal_state = True
                            self.object_detect.value = True

                # check cameras running
                if self.drone.get_state(camera_states)["active_cameras"] != 0 or self.thermal_state:
                    cams = "Healthy"
                elif cams != 'Unhealthy':
                    self.zmqmanager.log("Cameras are reporting UNHEALTHY!", level='ERROR')
                    cams = "Unhealthy"

                # check drone sensors
                sensors = self.drone.get_state(SensorsStatesListChanged)
                if sensors[Sensor.IMU]['sensorState'] == 1:
                    imu = "Healthy"
                else:
                    self.zmqmanager.log("IMU is reporting UNHEALTHY!", level='ERROR')
                    imu = "Unhealthy"

                if sensors[Sensor.magnetometer]['sensorState'] == 1:
                    mag = "Healthy"
                else:
                    self.zmqmanager.log("Magnetometer is reporting UNHEALTHY!", level='ERROR')
                    mag = "Unhealthy"

                if sensors[Sensor.barometer]['sensorState'] == 1:
                    baro = "Healthy"
                else:
                    self.zmqmanager.log("Barometer is reporting UNHEALTHY!", level='ERROR')
                    baro = "Unhealthy"

                if sensors[Sensor.GPS]['sensorState'] == 1:
                    gps = "Healthy"
                else:
                    self.zmqmanager.log("GPS is reporting UNHEALTHY!", level='ERROR')
                    gps = "Unhealthy"

                if sensors[Sensor.ultrasound]['sensorState'] == 1:
                    ultra = "Healthy"
                else:
                    self.zmqmanager.log("Ultrasound is reporting UNHEALTHY!", level='ERROR')
                    ultra = "Unhealthy"

                if sensors[Sensor.vertical_camera]['sensorState'] == 1:
                    vert = "Healthy"
                else:
                    self.zmqmanager.log("Vertical Camera is reporting UNHEALTHY!", level='ERROR')
                    vert = "Unhealthy"

                if self.state in [DroneState.Charging, DroneState.Idling, DroneState.Repositioning]:
                    position = self.drone.get_state(HomeChanged)
                    lat = position['latitude']
                    lng = position['longitude']
                elif self.state != DroneState.Error:
                    position = self.drone.get_state(PositionChanged)
                    lat = position['latitude']
                    lng = position['longitude']

                self.zmqmanager.log(f'Connected,{battery_level},{cams},{imu},{mag},{baro},{gps},{ultra},{vert},{lat},{lng}', level='SOH')
            else:
                self.zmqmanager.log(f'Disconnected,N/A,N/A,N/A,N/A,N/A,N/A,N/A,N/A,N/A,N/A', level='SOH')

            
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
                        if self.object_detect.value:
                            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

                            # apply a Gaussian blur to the image then find the brightest
                            # region
                            gray = cv2.GaussianBlur(gray, (41, 41), 0)
                            (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(gray)

                            if maxVal > 150.0:
                                cv2.circle(frame, maxLoc, 41, (255, 255, 255), 2)
                                #print(maxVal)
                                self.object_detected.value = True
                                self.object_px.value = maxLoc[0]
                                self.object_py.value = maxLoc[1]
                            else:
                                self.object_detected.value = False
                        
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
    
    def toggle_thermal(self):
        if not self.thermal_state:
            self.zmqmanager.log('Setting thermal mode to blended.')
            self.drone(set_mode(mode="blended")).wait().success()

            self.zmqmanager.log('Setting thermal mode to blended.')
            self.drone(set_rendering(mode="thermal", blending_rate=1)).wait().success()

            self.zmqmanager.log('Setting thermal palette to spot above threshold.')
            self.drone(set_palette_settings(mode="spot", 
                                     lowest_temp=305.372,
                                     highest_temp=316.483,
                                     outside_colorization="limited",
                                     relative_range="locked",
                                     spot_type="hot",
                                     spot_threshold=0.6)).wait().success()
        else:
            self.zmqmanager.log('Setting thermal mode to disabled.')
            self.drone(set_mode(mode="disabled")).wait().success()
            
            self.zmqmanager.log('Setting rendering mode to visible.')
            self.drone(set_rendering(mode="visible", blending_rate=0)).wait().success()

    def fly(self):
        self.survey_complete = False

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
        if battery_level < 25:
            self.state = DroneState.Charging
            self.zmqmanager.log('Drone needs to charge.', level='WARNING')
            return

        # check cameras running
        if self.drone.get_state(camera_states)["active_cameras"] == 0 and not self.thermal_state:
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

        position = self.drone.get_state(HomeChanged)
        self.start_lat = position['latitude']
        self.start_lng = position['longitude']
        self.start_alt = position['altitude']

        self.nonvolatile["home"] = [self.start_lat, self.start_lng]
        self.zmqmanager.log('Saving home position for return.')

        self.drone(moveTo(self.start_lat, self.start_lng, 10, MoveTo_Orientation_mode.TO_TARGET, 0, _timeout=1000)).wait().success()

        self.zmqmanager.log('Drone take off is successful going to survey.', level='SUCCESS')

        # TODO: determine if we need to land or go into idle if there's an error
        self.state = DroneState.Surveying

    def surveying(self):
        #maxtilt = self.drone.get_state(MaxTiltChanged)["max"]
        self.drone(MaxTilt(40)).wait()

        self.zmqmanager.log('Drone is generating a surveillance path.')
        survey_path = self.geofencemanager.get_survey_path([self.start_lat, self.start_lng])
        self.zmqmanager.log('Drone has a survey path.', level='SUCCESS')
        
        i = 1
        for p in survey_path:
            self.zmqmanager.log(f'Moving to survey waypoint {i} of {len(survey_path)}.')
            flying = self.drone(
                moveTo(p[0], p[1], 10, MoveTo_Orientation_mode.TO_TARGET, 0)
                >> moveToChanged(status='DONE', _timeout=500, _float_tol=(1e-05, 1e-06))
                )
            
            while not flying.success():
                if self.object_detected.value:
                    self.zmqmanager.log('Target detected!', level='LOG')
                    target_lat, target_lng = self.get_target_gps()

                    if self.geofencemanager.check_in_bounds([target_lat, target_lng]):
                        self.zmqmanager.log('Pursuing target!', level='LOG')
                        self.drone(CancelMoveTo()).wait().success()
                        self.state = DroneState.Chasing
                        return
                    else:
                        self.zmqmanager.log('Target out of bounds!', level='LOG')

            self.zmqmanager.log(f'Reached survey waypoint {i} of {len(survey_path)}!')
            i += 1

        self.zmqmanager.log('Drone has completed surveillance, no targets found.', level='SUCCESS')

        self.state = DroneState.Returning

    def get_target_gps(self):
        w = 320
        h = 256
        deg_pix = 0.15625   # 50 degrees HFOV 320 pixels

        px = self.object_px.value
        py = self.object_py.value

        rel_pitch = deg_pix * (px - (w / 2))
        rel_yaw = deg_pix * ((h - py) - (h / 2))

        gimbal_vec = self.drone.get_state(attitude)[0]

        target_pitch = gimbal_vec['pitch_absolute'] + rel_pitch
        target_yaw = gimbal_vec['yaw_absolute'] + rel_yaw

        position = self.drone.get_state(PositionChanged)

        length = position['altitude'] / math.tan(math.radians(target_pitch))

        return [position['latitude'] + (length * math.sin(math.radians(target_yaw))),
                position['longitude'] + (length * math.cos(math.radians(target_pitch)))]

    def chasing(self):
        updating = False
        while True:
            position = self.drone.get_state(PositionChanged)
            target_lat, target_lng = self.get_target_gps()
            target_path = self.geofencemanager.get_path([position['latitude'], position['longitude']],
                                                        [target_lat, target_lng])

            i = 1
            for p in target_path:
                self.zmqmanager.log(f'Moving to target waypoint {i} of {len(target_path)}.')
                flying = self.drone(
                    moveTo(p[0], p[1], 10, MoveTo_Orientation_mode.TO_TARGET, 0)
                    >> moveToChanged(status='DONE', _timeout=500, _float_tol=(1e-05, 1e-06))
                    )
                
                if self.object_detected.value:
                    self.zmqmanager.log('Target detected!', level='LOG')
                    target_lat, target_lng = self.get_target_gps()

                    if self.geofencemanager.check_in_bounds([target_lat, target_lng]):
                        self.zmqmanager.log('Updating target', level='LOG')
                        self.drone(CancelMoveTo()).wait().success()
                        updating = True
                        break
                    else:
                        self.zmqmanager.log('Target is out of bounds. Returning home.', level='LOG')
                        self.state = DroneState.Returning
                        return

                self.zmqmanager.log(f'Reached target waypoint {i} of {len(target_path)}!')
                i += 1

            if not updating:
                break

        self.zmqmanager.log('Reached targets last known location without updates returning home.', level='LOG')
        self.state = DroneState.Returning

    def returning(self):
        position = self.drone.get_state(PositionChanged)
        return_path = self.geofencemanager.get_path([position['latitude'], position['longitude']],
                                                    [self.start_lat, self.start_lng])

        self.zmqmanager.log('Drone has generated a return home path.', level='SUCCESS')

        i = 0
        for p in return_path:
            self.zmqmanager.log(f'Moving to return path waypoint {i} of {len(return_path)}.')
            self.drone(
                moveTo(p[0], p[1], 10, MoveTo_Orientation_mode.TO_TARGET, 0)
                >> moveToChanged(status='DONE', _timeout=500, _float_tol=(1e-05, 1e-06))
                ).wait().success()
            self.zmqmanager.log(f'Reached return path waypoint {i} of {len(return_path)}!')
            i += 1

        self.zmqmanager.log('Drone has made it home. Landing!', level='SUCCESS')

        self.state = DroneState.Landing

    def landing(self):
        self.drone(Landing() >> FlyingStateChanged(state="landed", _timeout=120)).wait().success()

        self.zmqmanager.log('Drone has successfully landed.', level='SUCCESS')

        self.state = DroneState.Idling

    def charging(self):
        self.state = DroneState.Idling
