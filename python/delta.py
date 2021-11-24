from __future__ import print_function
import serial
import threading
import time
import glob
import cv2
import numpy as np
from typing import NamedTuple
import math
import logging

import grpc
import camera_service_pb2
import camera_service_pb2_grpc


channel = grpc.insecure_channel('localhost:5555')
stub = camera_service_pb2_grpc.CameraDetectionServiceStub(channel)

serialPort = serial.Serial(
    port="/dev/ttyACM0", baudrate=115200, bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE
)

# LIFT_TIME = 0.15


def waitForReady():
  line = serialPort.readline().strip()
  while (line != b'ready'):
    print(line)
    time.sleep(0.005)
    line = serialPort.readline().strip()


waitForReady()
serialPort.write(bytes(f"X100 Y10\n", 'utf-8'))
waitForReady()
serialPort.write(bytes(f"B1\n", 'utf-8'))
waitForReady()


while True:
  time.sleep(0.05)

  response = stub.Detect(camera_service_pb2.CameraDetectionRequest())
  next_obj = None
  for obj in response.detected_objects:
    if 0 < obj.y:
      if next_obj is None or next_obj.y > obj.y:
        next_obj = obj

  if next_obj is None:
    time.sleep(1)
    continue

  print(next_obj.y)

  if next_obj.y > 100:
    serialPort.write(bytes(f"X{int(next_obj.x)}\n", 'utf-8'))
    waitForReady()

  # if next_obj.y < 5:
  #   serialPort.write(bytes(f"B0\n", 'utf-8'))
  if next_obj.y < 20:
    serialPort.write(bytes(f"M1 Y0\n", 'utf-8'))
    waitForReady()
    serialPort.write(bytes(f"Y10\n", 'utf-8'))
    waitForReady()
    if next_obj.length > 60:
      serialPort.write(bytes(f"X50\n", 'utf-8'))
    else:
      serialPort.write(bytes(f"X240\n", 'utf-8'))
    waitForReady()
    serialPort.write(bytes(f"M0\n", 'utf-8'))
    waitForReady()
