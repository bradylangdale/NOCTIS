
import zmq
from threading import Thread, Lock
import time

import csv
import math
import os
import queue
import shlex
import subprocess
import tempfile
import threading
import cv2
import numpy as np

import olympe
from olympe.messages.ardrone3.Piloting import TakeOff, Landing
from olympe.messages.ardrone3.Piloting import moveBy
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged
from olympe.messages.ardrone3.PilotingSettings import MaxTilt
from olympe.messages.ardrone3.PilotingSettingsState import MaxTiltChanged
from olympe.messages.ardrone3.GPSSettingsState import GPSFixStateChanged
from olympe.video.renderer import PdrawRenderer


class ZMQ_Manager:

    def __init__(self, drone):
        self.drone = drone
        self.running = True

        self.zmq_thread = Thread(target=self._zmqManager)
        self.zmq_thread.run()

    def _zmqManager(self):
        print("Connecting to the Web UI…")
        context = zmq.Context()
        socket = context.socket(zmq.REP)
        socket.bind("tcp://*:5555")

        while(self.running):
            message = socket.recv().decode("utf-8")

            #print(message)

            if message == 'Hello!':
                socket.send_string('Server Says Hi!')

                drone.stop()
                drone.start()
                t = threading.Thread(target=drone.fly)
                t.start()
            elif message == 'GetCurrentFrame':
                #try:
                    #print(drone.current_frame)
                if drone.current_frame != None:
                    socket.send_string(drone.current_frame.decode('iso-8859-1'))
                else:
                    socket.send_string('Not Ready')
            else:
                socket.send_string('Acknowledged')

        socket.close()

#!/usr/bin/env python

# NOTE: Line numbers of this example are referenced in the user guide.
# Don't forget to update the user guide after every modification of this example.


olympe.log.update_config({"loggers": {"olympe": {"level": "WARNING"}}})

DRONE_IP = os.environ.get("DRONE_IP", "10.202.0.1")
DRONE_RTSP_PORT = os.environ.get("DRONE_RTSP_PORT")


class StreamingExample:
    def __init__(self):
        # Create the olympe.Drone object from its IP address
        self.drone = olympe.Drone(DRONE_IP)
        #subprocess.run('rm -rf /home/noctis/NOCTIS/WebUI/wwwroot/Data/'.split(' '))
        subprocess.run('mkdir /home/noctis/NOCTIS/WebUI/wwwroot/Data/'.split(' '))
        subprocess.run('mkdir /home/noctis/NOCTIS/WebUI/wwwroot/Data/Videos/'.split(' '))
        self.tempd = '/home/noctis/NOCTIS/WebUI/wwwroot/Data/'#tempfile.mkdtemp(prefix="olympe_streaming_test_")
        print(f"Olympe streaming example output dir: {self.tempd}")

        self.video_thread = None
        self.running = False
        self.current_frame = None

    def start(self):
        # Connect to drone
        assert self.drone.connect(retry=3)
        self.running = True

        self.video_thread = threading.Thread(target=self.frame_processing)
        self.video_thread.start()

    def stop(self):
        assert self.drone.disconnect()
        self.running = False
        time.sleep(10)
        self.video_thread.join()

    def frame_processing(self):
        command = [
            'ffmpeg',
            '-i', 'rtsp://10.202.0.1/live',
            '-f', 'rawvideo',
            '-pix_fmt', 'bgr24',
            'pipe:'
        ]

        p = subprocess.Popen(command, stdout=subprocess.PIPE)

        width = 1280
        height = 720

        while self.running:

            try:
                raw_frame = p.stdout.read(width*height*3)
                frame = np.frombuffer(raw_frame, np.uint8)
                frame = frame.reshape((height, width, 3))

                # Use OpenCV to convert the yuv frame to RGB
                #cv2frame = cv2.cvtColor(yuv_frame.as_ndarray(), cv2_cvt_color_flag)
                self.current_frame = cv2.imencode(".jpg", frame)[1].tobytes()
                #cv2.imwrite('/home/brady/Projects/NOCTIS/WebUI/wwwroot/Data/current_frame.bmp', cv2frame)
                #count += 1
                
                #subprocess.run('ffmpeg -loglevel quiet -hide_banner -y -i ./WebUI/wwwroot/Videos/streaming.mp4 -g 52 -c:a aac -b:a 64k -b:v 448k -f mp4 -movflags frag_keyframe+empty_moov ./WebUI/wwwroot/Videos/frag.mp4'.split(' '))
            except Exception as e:
                p.kill()
                time.sleep(500)
                p = subprocess.Popen(command, stdout=subprocess.PIPE)
                print(e)

        p.kill()

    def fly(self):
        # Takeoff, fly, land, ...
        print("Takeoff if necessary...")
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
        maxtilt = self.drone.get_state(MaxTiltChanged)["max"]
        self.drone(MaxTilt(maxtilt)).wait()

        for i in range(4):
            print(f"Moving by ({i + 1}/4)...")
            self.drone(moveBy(10, 0, 0, math.pi, _timeout=20)).wait().success()

        print("Landing...")
        self.drone(Landing() >> FlyingStateChanged(state="landed", _timeout=5)).wait()
        print("Landed\n")


def test_streaming():
    streaming_example = StreamingExample()
    # Start the video stream
    streaming_example.start()
    # Perform some live video processing while the drone is flying
    streaming_example.fly()
    # Stop the video stream
    streaming_example.stop()
    # Recorded video stream postprocessing
    # streaming_example.replay_with_vlc()


if __name__ == "__main__":
    drone = StreamingExample()

    drone.start()
    zmqManager = ZMQ_Manager(drone)
