import math
import numpy as np
import cv2
import glob
import time

(camera_matrix, distortion) = (np.load("camera_matrix.npy"), np.load("distortion.npy"))

vid = cv2.VideoCapture(0)

def capture_photo():
  _, frame = vid.read()
  frame = cv2.undistort(frame, camera_matrix, distortion)
  return frame

# vid.release()
