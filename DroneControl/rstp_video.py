import cv2
import numpy as np
import subprocess


command = [
    'ffmpeg',
    '-i', 'rtsp://10.202.0.1/live',
    '-f', 'rawvideo',
    '-pix_fmt', 'bgr24',
    'pipe:'
]

p = subprocess.Popen(command, stdout=subprocess.PIPE)

while True:
    width = 1280
    height = 720

    raw_frame = p.stdout.read(width*height*3)
    frame = np.frombuffer(raw_frame, np.uint8)
    frame = frame.reshape((height, width, 3))

    cv2.imshow('stream', frame)
    cv2.waitKey(1)