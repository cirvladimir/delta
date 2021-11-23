import math
from typing import NamedTuple
import numpy as np
import cv2
import glob
import time
import threading
import serial

(camera_matrix, distortion) = (
    np.load("camera_matrix.npy"), np.load("distortion.npy"))


def capture_photo():
  vid = cv2.VideoCapture(0)

  vid.set(3, 1600)
  vid.set(4, 1200)
  _, frame = vid.read()
  frame = cv2.undistort(frame, camera_matrix, distortion)
  vid.release()
  return frame


# camera_rotation = cv2.Rodrigues(np.load("camera_rotation.npy"))[0]
camera_rotation = np.load("camera_rotation.npy")
camera_translation = np.load("camera_translation.npy")


def find_x_y(u, v):
  """Finds world X,Y coordinates given world Z and picture u,v coordinates.
  Arguments:
  u -- x coordinate of the pixel on the un-distorted image.
  v -- y coordinate of hte pixel on the un-distorted image.
  z -- world z coordinate. +z is closer to the camera
  Returns:
  (x, y) in world coordinates.
  """
  z = 0

  uv = np.array([[u], [v], [1]])
  inv_camera_rotation = np.linalg.inv(camera_rotation)
  inv_camera_matrix = np.linalg.inv(camera_matrix)
  lsm = np.dot(np.dot(inv_camera_rotation, inv_camera_matrix), uv)
  rsm = np.dot(inv_camera_rotation, camera_translation)

  s = (z + rsm[2, 0]) / lsm[2, 0]
  p = np.dot(inv_camera_rotation, s *
             np.dot(inv_camera_matrix, uv) - camera_translation)
  return np.array([p[0], p[1]])


def uv_to_xy(pt):
  mm_xy = find_x_y(pt[0], pt[1])
  return (mm_xy[0][0], mm_xy[1][0])

# vid.release()


class DetectedObject(NamedTuple):
  x: float
  y: float
  length: float
  time: float
  dx: float
  dy: float


def get_objects():
  # img = cv2.imread("a.png")
  img = capture_photo()
  image_time = time.time()

  img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

  COLOR_THRESH = 80

  scanned_points = set()

  def recursive_scan(x, y, scanned_points, img):
    point_glob = {(x, y)}
    next_points = [(x, y)]
    while len(next_points) > 0:
      (x, y) = next_points.pop()
      for y1 in range(y - 2, y + 3):
        for x1 in range(x - 2, x + 3):
          if (x1, y1) in scanned_points:
            continue
          scanned_points.add((x1, y1))
          if img[y1, x1] > COLOR_THRESH:
            next_points.append((x1, y1))
            point_glob.add((x1, y1))
    return point_glob

  point_globs = []

  start = time.time()

  for y in range(50, 1000):
    for x in range(100, 1200):
      if (x, y) in scanned_points:
        continue
      scanned_points.add((x, y))
      if img[y, x] > COLOR_THRESH:
        point_globs += [recursive_scan(x, y, scanned_points, img)]

  end = time.time()

  objects = []

  for point_glob in point_globs:
    if len(point_glob) < 40:
      continue
    min_x_pt = next(iter(point_glob))
    max_x_pt = next(iter(point_glob))
    min_y_pt = next(iter(point_glob))
    max_y_pt = next(iter(point_glob))
    avg_x = 0
    avg_y = 0
    for pt in point_glob:
      avg_x += pt[0]
      avg_y += pt[1]
      if min_x_pt[0] > pt[0]:
        min_x_pt = pt
      if max_x_pt[0] < pt[0]:
        max_x_pt = pt
      if min_y_pt[1] > pt[1]:
        min_y_pt = pt
      if max_y_pt[1] < pt[1]:
        max_y_pt = pt

    avg_x /= len(point_glob)
    avg_y /= len(point_glob)

    def my_dist(p1, p2):
      p1 = uv_to_xy(p1)
      p2 = uv_to_xy(p2)
      return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)
    max_dist = 0
    edge_pts = [min_x_pt, max_x_pt, min_y_pt, max_y_pt]
    for i in range(0, 3):
      for j in range(i, 4):
        max_dist = max(max_dist, my_dist(edge_pts[i], edge_pts[j]))

    mm_xy = find_x_y(avg_x, avg_y)

    objects.append(DetectedObject(
        mm_xy[0][0], mm_xy[1][0], max_dist, image_time, 0, 0))

  # print(objects)

  # print(end - start)

  # print(len(objects))

  # new_img = np.zeros((1200, 1600, 3), np.uint8)

  # color = 432532
  # MAX_COLOR = 256 * 256 * 256

  # for point_glob in point_globs:
  #   if len(point_glob) < 40:
  #     continue
  #   color_ar = [color % 256, (color // 256) % 256, (color // (256 * 256)) % 256]
  #   for pt in point_glob:
  #     # if new_img[pt[1], pt[0], :] != [0, 0, 0]:
  #     #   print("error")
  #     new_img[pt[1], pt[0], :] = color_ar
  #   color = (color + 423237) * 23 % MAX_COLOR

  # cv2.imwrite("b.png", new_img)
  return objects


# mm per second
CONVEYOR_SPEED = 60

last_objects = []
objects_lock = threading.Lock()


def objects_updater():
  global last_objects, objects_lock
  while True:
    detected_objects = get_objects()
    tmp_last_objects = list(last_objects)
    new_detected_objects = []
    for detected_object in detected_objects:
      found_object = None
      for last_object in tmp_last_objects:
        expected_x = last_object.x
        expected_y = last_object.y - CONVEYOR_SPEED * \
            (detected_object.time - last_object.time)
        if (abs(expected_x - detected_object.x) < 10 and
            abs(expected_y - detected_object.y) < 10 and
                abs(last_object.length - detected_object.length) < 10):
          found_object = last_object
          break
      if found_object:
        tmp_last_objects.remove(found_object)
        new_detected_objects.append(DetectedObject(detected_object.x, detected_object.y,
                                                   detected_object.length, detected_object.time,
                                                   dx=(
                                                       (detected_object.x - last_object.x) / (detected_object.time - last_object.time)),
                                                   dy=((detected_object.y - last_object.y) / (detected_object.time - last_object.time))))
      else:
        new_detected_objects.append(detected_object)

    tmp_last_objects = [
        last_object for last_object in tmp_last_objects if last_object.y < 300]

    tmp_last_objects += detected_objects
    with objects_lock:
      last_objects = tmp_last_objects
      print(last_objects)


serialPort = serial.Serial(
    port="/dev/ttyACM0", baudrate=115200, bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE
)

threading.Thread(target=objects_updater, daemon=True).start()


def waitForReady():
  line = serialPort.readline().strip()
  while (line != b'ready'):
    print(line)
    time.sleep(0.005)
    line = serialPort.readline().strip()


waitForReady()
serialPort.write(bytes(f"B1\n", 'utf-8'))
waitForReady()


while True:
  time.sleep(0.05)
  pickup_obj = None
  with objects_lock:
    old_objects = []
    for last_object in last_objects:
      expected_x = last_object.x
      expected_y = last_object.y - CONVEYOR_SPEED * \
          (time.time() - last_object.time)

      # print((expected_x, expected_y))

      if expected_y < 0:
        old_objects.append(last_object)
      elif expected_y < 100:
        pickup_obj = last_object
        break
    for old_object in old_objects:
      last_objects.remove(old_object)

  if pickup_obj:
    expected_x = pickup_obj.x
    serialPort.write(bytes(f"X{int(expected_x)} Y15\n", 'utf-8'))
    waitForReady()
    expected_y = pickup_obj.y - CONVEYOR_SPEED * \
        (time.time() - pickup_obj.time)
    time.sleep(max(expected_y / CONVEYOR_SPEED, 0))
    serialPort.write(bytes(f"M1 Y0\n", 'utf-8'))
    waitForReady()
    serialPort.write(bytes(f"Y15\n", 'utf-8'))
    waitForReady()
    if pickup_obj.length > 40:
      serialPort.write(bytes(f"X50\n", 'utf-8'))
    else:
      serialPort.write(bytes(f"X240\n", 'utf-8'))
    waitForReady()
    serialPort.write(bytes(f"M0\n", 'utf-8'))
    waitForReady()
    time.sleep(0.5)
