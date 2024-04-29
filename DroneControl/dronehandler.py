from threading import Thread
import time

import math
import os
import subprocess
import cv2
import numpy as np
from enum import IntEnum

os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "rtsp_transport;udp"

from scipy.spatial.transform import Rotation
import olympe
import olympe_deps as od
from olympe.messages.skyctrl.CoPiloting import setPilotingSource
from olympe.messages.ardrone3.Piloting import (TakeOff, Landing, 
                                               moveBy, moveTo, CancelMoveTo,
                                               StartPilotedPOIV2, StopPilotedPOI)
from olympe.messages.ardrone3.PilotingState import (FlyingStateChanged, 
                                                    AltitudeAboveGroundChanged, 
                                                    PositionChanged,
                                                    SpeedChanged,
                                                    moveToChanged,
                                                    moveByChanged)
from olympe.messages.ardrone3.PilotingSettings import MaxTilt
from olympe.messages.ardrone3.PilotingSettingsState import MaxTiltChanged
from olympe.messages.ardrone3.GPSSettingsState import GPSFixStateChanged, HomeChanged
from olympe.messages.common.CommonState import BatteryStateChanged, SensorsStatesListChanged
from olympe.messages.camera import camera_states
from olympe.messages.gimbal import attitude, set_target

from olympe.enums.common.CommonState import SensorsStatesListChanged_SensorName as Sensor
from olympe.enums.ardrone3.Piloting import MoveTo_Orientation_mode
from olympe.messages.thermal import set_mode, set_rendering, set_palette_settings, mode, rendering

from multiprocessing import Process, Manager
from ctypes import c_bool, c_uint16, c_int8

import json

# Load the YOLOv8 model
from ultralytics import YOLO


olympe.log.update_config({"loggers": {"olympe": {"level": "WARNING"}}})

RPI = False
SIM = True

if SIM:
    DRONE_IP = os.environ.get("DRONE_IP", "10.202.0.1")
else:
    DRONE_IP = os.environ.get("DRONE_IP", "192.168.53.1")

DRONE_RTSP_PORT = os.environ.get("DRONE_RTSP_PORT", '554')

JPG_QUALITY = [int(cv2.IMWRITE_JPEG_QUALITY), 40]

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


class CameraMode(IntEnum):
    Init = -1
    Visible = 0
    Thermal = 1
    VisibleDetect = 2
    ThermalDetect = 3


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
        self.current_frame = self.manager.list()
        self.current_frame.append([])
        self.write_ready = True

        self.display_num = self.manager.Value(c_int8, -1)

        self.need_video_restart = False
        self.video_thread = None

        self.running = self.manager.Value(c_bool, False)
        self.object_detected = self.manager.Value(c_bool, False)
        self.object_px = self.manager.Value(c_uint16, 0)
        self.object_py = self.manager.Value(c_uint16, 0)
        self.camera_mode = self.manager.Value(c_uint16, 0)

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

            try:
                self.start_lat = self.nonvolatile['home'][0]
                self.start_lng = self.nonvolatile['home'][1]
            except:
                self.zmqmanager.log('No home position found in the dcs.json', level='WARNING')
            
            self.zmqmanager.log('Successfully loaded dcs.json', level='SUCCESS')
        except:
            self.zmqmanager.log('No dcs.json found creating one.', level='WARNING')

            with open('../DroneControl/data/dcs.json', 'w') as f:
                json.dump(self.nonvolatile, f)

        if not(self.state == DroneState.Idling or self.state == DroneState.Charging):
            self.state = DroneState.Error

        # Load the visible model
        self.visible_model = YOLO('../DroneControl/models/yolov5-visible.onnx', verbose=False)

        # Load the thermal model
        self.thermal_model = YOLO('../DroneControl/models/yolov5-thermal.onnx', verbose=False)

        Thread(target=self.heartbeat).start()

        self.flight_thread = None

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
                    #self.shared_frames[:] = []
                    Thread(target=self.stop).start()

            if not self.drone.connection_state():
                time.sleep(0.5)
                self.zmqmanager.log('Attempting to reconnect!', level='WARNING')
                self.start(reattempt=True)

            if self.drone.connection_state():
                if self.object_detected.value:
                    target = self.get_target_gps()
                    self.zmqmanager.log(str(target[0]) + "," + str(target[1]), level='TARGET')

                # check battery level
                battery_level = self.drone.get_state(BatteryStateChanged)["percent"]

                if battery_level % 10 == 0 or battery_level != last_bat:
                    if battery_level % 10 == 0 and battery_level != last_bat:
                        self.zmqmanager.log('Battery Level at ' + str(battery_level) + '%', level='WARNING')

                    if battery_level != last_bat and battery_level <= 10:
                        if self.state == DroneState.Idling and self.state != DroneState.Charging:
                            if self.flight_thread is not None:
                                self.state = DroneState.Idling
                                self.flight_thread.join()

                            self.state = DroneState.Charging
                            self.flight_thread = Thread(target=self.fly)
                            self.flight_thread.start()
                            self.zmqmanager.log('Starting drone charging.', level='LOG')
                        elif self.state == DroneState.Surveying or self.state == DroneState.Chasing:
                            self.zmqmanager.log('Battery Level is too low canceling current flight.', level='WARNING')
                            self.state = DroneState.Error
                            self.zmqmanager.log('Drone Control requires full restart. Restarting now!', level='WARNING')
                
                    last_bat = battery_level
                
                if self.state == DroneState.Error:
                    self.zmqmanager.log('Drone Control reset during flight, forcing drone to return home!', level='ERROR')

                    if self.flight_thread is not None:
                        self.state = DroneState.Idling
                        self.flight_thread.join()
                    
                    self.state = DroneState.Returning
                    self.flight_thread = Thread(target=self.fly)
                    self.flight_thread.start()

                self.nonvolatile['state'] = int(self.state)

                with open('../DroneControl/data/dcs.json', 'w') as f:
                    json.dump(self.nonvolatile, f)

                if not SIM:
                    if str(self.drone.get_state(mode)["mode"]) == "mode.disabled" or \
                    str(self.drone.get_state(rendering)["mode"]) == "rendering_mode.visible":
                        if self.thermal_state != False:
                            self.zmqmanager.log('Thermal Imaging is disabled.')
                            self.thermal_state = False

                            if self.camera_mode.value == int(CameraMode.ThermalDetect):
                                self.camera_mode.value = int(CameraMode.VisibleDetect)
                            else:
                                self.camera_mode.value = int(CameraMode.Visible)
                    else:
                        if self.thermal_state != True:
                            self.zmqmanager.log('Thermal Imaging is enabled.')
                            self.thermal_state = True

                            if self.camera_mode.value == int(CameraMode.VisibleDetect):
                                self.camera_mode.value = int(CameraMode.ThermalDetect)
                            else:
                                self.camera_mode.value = int(CameraMode.Thermal)

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
        FRAMERATE = 30
        vcap = cv2.VideoCapture(f'rtsp://{DRONE_IP}:{DRONE_RTSP_PORT}/live')
        vcap.set(cv2.CAP_PROP_FPS, FRAMERATE)
        vcap.set(cv2.CAP_PROP_FRAME_WIDTH, WEBCAM_RAW_RES[0])
        vcap.set(cv2.CAP_PROP_FRAME_HEIGHT, WEBCAM_RAW_RES[1])

        last_mode = CameraMode.Init

        write_num = 1

        while self.running.value:
            if last_mode != CameraMode(self.camera_mode.value):
                current_mode = CameraMode(self.camera_mode.value)
                print('Switched from', last_mode, 'to', current_mode)
                if (last_mode in [CameraMode.Visible, CameraMode.VisibleDetect] and current_mode in [CameraMode.Thermal, CameraMode.ThermalDetect]) or \
                   (last_mode in [CameraMode.Thermal, CameraMode.ThermalDetect] and current_mode in [CameraMode.Visible, CameraMode.VisibleDetect]):
                    print('Video is restarting')
                    vcap.release()
                    vcap.open(f'rtsp://{DRONE_IP}:{DRONE_RTSP_PORT}/live')
                else:
                    print('Video is not restarting')

                last_mode = current_mode

            ret, frame = vcap.read()

            if ret == True and self.write_ready:
                write_num += 1;
                write_num %= 32;

                self.write_ready = False
                Thread(target=self.process_frame, args=(write_num, frame)).start()
    
    def process_frame(self, num, frame):
        current_mode = CameraMode(self.camera_mode.value)

        if current_mode == CameraMode.Visible:
            time.sleep(0.1)
        elif current_mode == CameraMode.VisibleDetect:
            results = self.visible_model.predict(source=frame, imgsz=224, verbose=False)

            if len(results[0].boxes) > 0:
                #print(results[0].boxes)
                target = results[0].boxes[0].xyxy[0]
                self.object_detected.value = True

                self.object_px.value = int((target[0] + target[2]) / 2)
                self.object_py.value = int(target[3])

                frame = results[0].plot()
            else:
                self.object_detected.value = False

            time.sleep(0.25)
        elif current_mode == CameraMode.Thermal:
            x1 = 240
            y1 = 30
            x2 = 1020
            y2 = 640

            frame = frame[y1:y2, x1:x2]
        elif current_mode == CameraMode.ThermalDetect:
            x1 = 240
            y1 = 30
            x2 = 1020
            y2 = 640

            frame = frame[y1:y2, x1:x2]

            results = self.visible_model.predict(source=frame, imgsz=224, verbose=False)

            if len(results[0].boxes) > 0:
                target = results[0].boxes[0].xyxy[0]
                self.object_detected.value = True

                self.object_px.value = int((target[0] + target[2]) / 2)
                self.object_py.value = int(target[3])

                frame = results[0].plot()
            else:
                self.object_detected.value = False

        self.current_frame[0] = frame
        cv2.imwrite('./wwwroot/Data/Videos/current_frame' + str(num) + '.jpg', frame, JPG_QUALITY)
        self.display_num.value = num

        self.write_ready = True

    def toggle_thermal(self):
        if not self.thermal_state:
            self.zmqmanager.log('Setting thermal mode to blended.')
            self.drone(set_mode(mode="blended")).wait().success()

            self.zmqmanager.log('Setting thermal mode to blended.')
            self.drone(set_rendering(mode="thermal", blending_rate=1)).wait().success()

            self.zmqmanager.log('Setting thermal palette to spot above threshold.')
            self.drone(set_palette_settings(mode="relative", 
                                     lowest_temp=0,
                                     highest_temp=1000,
                                     outside_colorization="limited",
                                     relative_range="unlocked",
                                     spot_type="hot",
                                     spot_threshold=0.5)).wait().success()
        else:
            self.zmqmanager.log('Setting thermal mode to disabled.')
            self.drone(set_mode(mode="disabled")).wait().success()
            
            self.zmqmanager.log('Setting rendering mode to visible.')
            self.drone(set_rendering(mode="visible", blending_rate=0)).wait().success()

    def toggle_detection(self):
        mode = CameraMode(self.camera_mode.value)

        if mode == CameraMode.Visible:
            self.camera_mode.value = int(CameraMode.VisibleDetect)
            self.zmqmanager.log('Enabling visual target detection!', level='LOG')
        elif mode == CameraMode.VisibleDetect:
            self.camera_mode.value = int(CameraMode.Visible)
            self.zmqmanager.log('Disabling visual target detection!', level='LOG')
            self.object_detected.value = False
        elif mode == CameraMode.Thermal:
            self.camera_mode.value = int(CameraMode.ThermalDetect)
            self.zmqmanager.log('Enabling thermal target detection!', level='LOG')
        elif mode == CameraMode.ThermalDetect:
            self.camera_mode.value = int(CameraMode.Thermal)
            self.zmqmanager.log('Disabling thermal target detection!', level='LOG')
            self.object_detected.value = False

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
        if battery_level < 5:
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

        #if not self.thermal_state:
        #    self.toggle_thermal()
        #    time.sleep(30)
        if not CameraMode(self.camera_mode.value) in [CameraMode.VisibleDetect, CameraMode.ThermalDetect]:
            self.toggle_detection()
            time.sleep(10)

        self.zmqmanager.log('Drone is taking off.')
        self.nonvolatile["start_yaw"] = self.drone.get_state(attitude)[0]['yaw_absolute']

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

        self.drone(moveTo(self.start_lat, self.start_lng, 10, MoveTo_Orientation_mode.TO_TARGET, 0, _timeout=10)).wait().success()

        time.sleep(5)

        self.zmqmanager.log('Drone take off is successful going to survey.', level='SUCCESS')

        self.state = DroneState.Surveying

    def surveying(self):
        self.drone(MaxTilt(40)).wait()

        self.drone(set_target(gimbal_id=0,
                                                  control_mode='position',
                                                  yaw_frame_of_reference='relative',
                                                  yaw=0.0,
                                                  pitch_frame_of_reference='relative',
                                                  pitch=-35,
                                                  roll_frame_of_reference='relative',
                                                  roll=0.0,
                                              )).wait().success()

        self.zmqmanager.log('Drone is generating a surveillance path.')
        survey_path = self.geofencemanager.get_survey_path([self.start_lat, self.start_lng])
        self.zmqmanager.log('Drone has a survey path.', level='SUCCESS')
        
        i = 1
        for p in survey_path:
            self.zmqmanager.log(f'Moving to survey waypoint {i} of {len(survey_path)}.')
            flying = self.drone(
                moveTo(p[0], p[1], 5, MoveTo_Orientation_mode.TO_TARGET, 0)
                >> moveToChanged(status='DONE', _timeout=500, _float_tol=(1e-05, 1e-06))
                )
            
            while not flying.success():
                if self.state != DroneState.Surveying:
                    self.zmqmanager.log('The Surveying state was interrupted.', level='WARNING')
                    return

                if self.object_detected.value:
                    self.zmqmanager.log('Target detected!', level='LOG')
                    target_coord = self.get_target_gps()

                    if self.geofencemanager.check_in_bounds(target_coord):
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

    # TODO: adjust w, h, deg_pix for thermal/visible camera
    def get_target_gps(self):
        # thermal
        #w = 780
        #h = 610
        w = 1280
        h = 720
        deg_pix = 50 / 1280.0
        #deg_pix = 0.064102564   # 50 degrees HFOV 780 pixels

        px = self.object_px.value
        py = self.object_py.value

        rel_yaw = deg_pix * (px - (w / 2))
        rel_pitch = deg_pix * ((h / 2) - py)

        gimbal_vec = self.drone.get_state(attitude)[0]

        print(rel_pitch)
        print(rel_yaw)

        target_pitch = min(90 - abs(gimbal_vec['pitch_absolute'] + rel_pitch), 85)
        target_yaw = gimbal_vec['yaw_absolute'] + rel_yaw

        position = self.drone.get_state(PositionChanged)
        
        length = abs(position['altitude'] * math.tan(math.radians(target_pitch)))

        dy = length * math.sin(math.radians(target_yaw))
        dx = length * math.cos(math.radians(target_yaw))

        latitude  = position['latitude']  - (dy / 111111.0)
        longitude = position['longitude'] - (dx / (111111.0 * math.cos(math.radians(position['latitude']))))

        return [ latitude, longitude ]

    def get_tracking_pos(self, target, position, radius=4):
        r = radius / 111111.0

        dx = position['latitude'] - target[0]
        dy = position['longitude'] - target[1]

        mag = math.sqrt(math.pow(dx, 2) + math.pow(dy, 2))

        return [((dx / mag) * r) + target[0], ((dy / mag)) * r + target[1]]

    def chasing(self):
        target = self.get_target_gps()
        position = self.drone.get_state(PositionChanged)
        tracking_pos = self.get_tracking_pos(target, position)

        detected = True
        while detected:
            detected = False

            self.zmqmanager.log(f'Moving to target position.')
            self.drone(StopPilotedPOI()).wait().success()
            flying = self.drone(
                moveTo(tracking_pos[0], tracking_pos[1], 5, MoveTo_Orientation_mode.TO_TARGET, 0)
                >> moveToChanged(status='DONE', _timeout=500, _float_tol=(1e-05, 1e-06))
                )

            while not flying.success():
                if self.object_detected.value:
                    target = self.get_target_gps()

                    if self.geofencemanager.check_in_bounds(target):
                        self.zmqmanager.log('Target Position Updated.', level='LOG')
                        self.drone(CancelMoveTo()).wait().success()

                        position = self.drone.get_state(PositionChanged)
                        tracking_pos = self.get_tracking_pos(target, position)

                        detected = True
                        break

            if not detected:
                self.zmqmanager.log(f'Reached target position!')
                self.drone(StartPilotedPOIV2(target[0], target[1], 1, mode='locked_gimbal')).wait().success()
                self.zmqmanager.log(f'Scanning for target...')
                time.sleep(3)

                for j in range(100):
                    if self.object_detected.value:
                        target = self.get_target_gps()
                        position = self.drone.get_state(PositionChanged)
                        tracking_pos = self.get_tracking_pos(target, position)
                        
                        self.zmqmanager.log('Target position updated!', level='LOG')
                        detected = True
                        break
                    else:
                        time.sleep(0.25)

                if not self.geofencemanager.check_in_bounds(target):
                    self.zmqmanager.log('Target has left the geofence! Returning home.', level='SUCCESS')
                    self.drone(StopPilotedPOI()).wait().success()
                    self.state = DroneState.Returning
                    return

            if self.state != DroneState.Chasing:
                self.zmqmanager.log('The Chasing state was interrupted.', level='WARNING')
                return

        self.zmqmanager.log('Reached targets last known location without updates returning home.', level='LOG')
        self.drone(StopPilotedPOI()).wait().success()
        self.state = DroneState.Returning

    def returning(self):
        if self.thermal_state:
            self.toggle_thermal()

        position = self.drone.get_state(PositionChanged)
        return_path = self.geofencemanager.get_path([position['latitude'], position['longitude']],
                                                    [self.start_lat, self.start_lng])

        self.zmqmanager.log('Drone has generated a return home path.', level='SUCCESS')

        i = 1
        for p in return_path:
            self.zmqmanager.log(f'Moving to return path waypoint {i} of {len(return_path)}.')
            self.drone(
                moveTo(p[0], p[1], 10, MoveTo_Orientation_mode.TO_TARGET, 0)
                >> moveToChanged(status='DONE', _timeout=500, _float_tol=(1e-05, 1e-06))
                ).wait().success()
            self.zmqmanager.log(f'Reached return path waypoint {i} of {len(return_path)}!')
            i += 1

            if self.state != DroneState.Returning:
                self.zmqmanager.log('The Returning state was interrupted.', level='WARNING')
                return

        self.zmqmanager.log('Drone has made it home. Landing!', level='SUCCESS')

        self.state = DroneState.Landing

    def landing(self):
        if CameraMode(self.camera_mode.value) in [CameraMode.VisibleDetect, CameraMode.ThermalDetect]:
            self.toggle_detection()

        self.zmqmanager.log('Drone is adjusting the camera.', level='LOG')

        self.drone(set_target(gimbal_id=0,
                    control_mode='position',
                    yaw_frame_of_reference='relative',
                    yaw=0.0,
                    pitch_frame_of_reference='relative',
                    pitch=-90,
                    roll_frame_of_reference='relative',
                    roll=0.0,
                )).wait().success()
        
        time.sleep(10)

        # adjust aruco stuff here
        arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        arucoParams = cv2.aruco.DetectorParameters()
        
        if not RPI:
            detector = cv2.aruco.ArucoDetector(arucoDict, arucoParams)

        markerSizeInM = 0.152
        mtx = np.array([[ 931.35139613, 0, 646.14084186 ],
              [ 0, 932.19736807, 370.42202449 ],
                        [ 0, 0, 1 ]])
        dist = np.array([[ 0.01386571, -0.00697705,  0.00331248,  0.00302185, -0.04361382]])
        
        corrections = 3
        decent_amount = 12.0 / corrections
        for i in range(corrections):
            self.zmqmanager.log('Drone scanning for an AruCo marker.', level='LOG')
            
            detections = 0
            dx = 0
            dy = 0
            dyaw = 0
            for j in range(25):
                # detect ArUco markers in the input frame
                if not RPI:
                    (corners, ids, rejected) = detector.detectMarkers(self.current_frame[0])
                else:
                    (corners, ids, rejected) = cv2.aruco.detectMarkers(self.current_frame[0], arucoDict, parameters=arucoParams)

                if ids is not None and len(ids) > 0:
                    _dx, _dy, _dyaw = self.estimatePoseSingleMarkers(corners, markerSizeInM, mtx, dist)
                    dx += _dx
                    dy += _dy
                    dyaw += _dyaw
                    detections += 1

                time.sleep(0.05)

            if detections > 0:
                dx /= detections
                dy /= detections
                dyaw /= detections

                self.zmqmanager.log('AruCo marker scan completed with ' + str(detections) + ' detections! Correcting now.', level='SUCCESS')

                self.drone(
                    moveBy(dx - 0.02, dy, decent_amount, dyaw)
                    >> moveByChanged(status='DONE', _timeout=10, _float_tol=(1e-05, 1e-06))
                ).wait().success()
            
                self.zmqmanager.log('Correction ' + str(i + 1) + ' of ' + str(corrections) + ' complete!', level='SUCCESS')
            else:
                self.zmqmanager.log('AruCo marker not found! Lowering height.', level='WARNING')
                
                self.drone(
                    moveTo(self.start_lat, self.start_lng, decent_amount,
                           MoveTo_Orientation_mode.HEADING_DURING,
                           math.radians(self.nonvolatile['start_yaw'] - self.drone.get_state(attitude)[0]['yaw_absolute']))
                    >> moveToChanged(status='DONE', _timeout=10, _float_tol=(1e-05, 1e-06))
                ).wait().success()

            if SIM:
                time.sleep(3)
            else:
                time.sleep(1)

        self.zmqmanager.log('Drone is landing!', level='LOG')

        self.drone(Landing() >> FlyingStateChanged(state="landed", _timeout=120)).wait().success()

        self.zmqmanager.log('Drone has successfully landed.', level='SUCCESS')

        self.drone(set_target(gimbal_id=0,
            control_mode='position',
            yaw_frame_of_reference='relative',
            yaw=0.0,
            pitch_frame_of_reference='relative',
            pitch=0.0,
            roll_frame_of_reference='relative',
            roll=0.0,
        )).wait().success()

        self.state = DroneState.Idling

    def charging(self):
        self.state = DroneState.Idling

    def euler_from_quaternion(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
            
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
            
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
            
        return roll_x, pitch_y, yaw_z # in radians

    def estimatePoseSingleMarkers(self, corners, marker_size, mtx, distortion):
        '''
        This will estimate the rvec and tvec for each of the marker corners detected by:
        corners, ids, rejectedImgPoints = detector.detectMarkers(image)
        corners - is an array of detected corners for each detected marker in the image
        marker_size - is the size of the detected markers
        mtx - is the camera matrix
        distortion - is the camera distortion matrix
        RETURN list of rvecs, tvecs, and trash (so that it corresponds to the old estimatePoseSingleMarkers())
        '''

        if not RPI:
            marker_points = np.array([[-marker_size / 2, marker_size / 2, 0],
                                    [marker_size / 2, marker_size / 2, 0],
                                    [marker_size / 2, -marker_size / 2, 0],
                                    [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)
            trash = []
            rvecs = []
            tvecs = []
            
            for c in corners:
                nada, R, t = cv2.solvePnP(marker_points, c, mtx, distortion, False, cv2.SOLVEPNP_IPPE_SQUARE)
                rvecs.append([[R[0][0], R[1][0], R[2][0]]])
                tvecs.append([[t[0][0], t[1][0], t[2][0]]])
                trash.append(nada)
        else:
            rvecs , tvecs, trash = cv2.aruco.estimatePoseSingleMarkers(corners, marker_size, mtx, distortion)

        # Store the rotation information
        rotation_matrix = np.eye(4)
        rotation_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs[0][0]))[0]
        r = Rotation.from_matrix(rotation_matrix[0:3, 0:3])
        quat = r.as_quat()   
         
        # Quaternion format     
        transform_rotation_x = quat[0] 
        transform_rotation_y = quat[1] 
        transform_rotation_z = quat[2] 
        transform_rotation_w = quat[3] 
         
        # Euler angle format in radians
        roll_x, pitch_y, yaw_z = self.euler_from_quaternion(transform_rotation_x, 
                                                       transform_rotation_y, 
                                                       transform_rotation_z, 
                                                       transform_rotation_w)
        
        #yaw_z += math.pi

        # reduce the angle  
        yaw_z =  yaw_z % (2 * math.pi); 

        # force it to be the positive remainder, so that 0 <= angle < 360  
        yaw_z = (yaw_z + (2 * math.pi)) % (2 * math.pi);  

        # force into the minimum absolute value residue class, so that -180 < angle <= 180  
        if (yaw_z > math.pi):
            yaw_z -= 2 * math.pi; 

        rvecs[0][0][2] = yaw_z
        
        return -tvecs[0][0][1], tvecs[0][0][0], rvecs[0][0][2]
