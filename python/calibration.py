import math
import numpy as np
import cv2
import glob
import time

# Size of the checker board counting number of squares.
CALIBRATION_GRID_SIZE = (8, 10)
# Size in mm of each square in the checker board.
CALIBRATION_SQUARE_SIZE_MM = 20

# Capture images:
# vid = cv2.VideoCapture(0)
# vid.set(3, 1600)
# vid.set(4, 1200)

# for i in range(0, 160):
#   _, frame = vid.read()

#   cv2.imwrite(f"images/calib_{i}.png", frame)
#   cv2.imwrite(f"a.png", frame)
#   time.sleep(0.4)

# vid.release()


def calibrate_camera(file_paths):
  # Defining the dimensions of checkerboard
  checkerboard_size = (
      CALIBRATION_GRID_SIZE[0] - 1, CALIBRATION_GRID_SIZE[1] - 1)

  # Creating vector to store vectors of 3D points for each checkerboard image
  world_points = []
  # Creating vector to store vectors of 2D points for each checkerboard image
  image_points = []

  # Defining the world coordinates for 3D points
  grid_points = np.zeros(
      (1, checkerboard_size[0] * checkerboard_size[1], 3), np.float32)
  grid_points[0, :, :2] = np.mgrid[0:checkerboard_size[0],
                                   0:checkerboard_size[1]].T.reshape(-1, 2)

  # Extracting path of individual image stored in a given directory
  images = glob.glob(file_paths)
  for file_name in images:
    img = cv2.imread(file_name)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # Find the chess board corners
    # If desired number of corners are found in the image then ret = true
    ret, corners = cv2.findChessboardCorners(
        gray, checkerboard_size, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)

    if ret == True:
      world_points.append(grid_points)
      # refining pixel coordinates for given 2d points.
      refined_corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1),
                                         (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))

      image_points.append(refined_corners)

  print(f"Calibration images: {len(world_points)}")
  ret, camera_matrix, distortion, _, _ = cv2.calibrateCamera(
      world_points, image_points, gray.shape[::-1], None, None)

  return (camera_matrix, distortion)


def make_rotation_matrix(angle):
  c = math.cos(angle)
  s = math.sin(angle)
  return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], np.float32)


def get_aruco_corners(point_1, point_2):
  # Note, these values were taken from the doc from the calibration sheet.
  # Given point 1 and point 2, where are the corners?
  default_corners = np.array([
      # Marker 0:
      [[0, 0, 0],
       [0, 100, 0],
       [100, 100, 0],
       [100, 0, 0]],

      # Marker 1:
      [[142, 0, 0],
       [142, 100, 0],
       [242, 100, 0],
       [242, 0, 0]],
  ], np.float32)

  known_point_1 = np.array([[32 - 20], [200 - 15], [0]], np.float32)
  known_point_2 = np.array([[255.5 - 20], [197.5 - 15], [0]], np.float32)

  z_rotation = (math.atan2(point_2[1][0] - point_1[1][0], point_2[0][0] - point_1[0][0]) -
                math.atan2(known_point_2[1][0] - known_point_1[1][0], known_point_2[0][0] - known_point_1[0][0]))
  rotation_matrix = make_rotation_matrix(z_rotation)
  translation = point_1 - rotation_matrix @ known_point_1
  return np.array([
      [(translation + rotation_matrix @ pt.reshape((3, 1))).reshape((3,))
       for pt in rect]
      for rect in default_corners], np.float32)


def find_camera_transform_aruco(camera_matrix, distortion, image,
                                point_1, point_2):
  arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_50)
  arucoParams = cv2.aruco.DetectorParameters_create()
  (corners, ids, _) = cv2.aruco.detectMarkers(
      image, arucoDict, parameters=arucoParams)

  board_corners = get_aruco_corners(point_1, point_2)
  # print(board_corners)
  aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_50)
  board = cv2.aruco.Board_create(
      board_corners, aruco_dict, np.array([0, 1]))

  (_, rvec, tvec) = cv2.aruco.estimatePoseBoard(
      corners, ids, board, camera_matrix, distortion, None, None)

  return (rvec, tvec)

# (camera_matrix, distortion) = calibrate_camera("images/*.png")
# print(camera_matrix)
# print(distortion)
# np.save("camera_matrix.npy", camera_matrix)
# np.save("distortion.npy", distortion)


(camera_matrix, distortion) = (
    np.load("camera_matrix.npy"), np.load("distortion.npy"))
# print(camera_matrix)
# print(distortion)


vid = cv2.VideoCapture(0)

vid.set(3, 1600)
vid.set(4, 1200)

_, frame = vid.read()
frame = cv2.undistort(frame, camera_matrix, distortion)

cv2.imwrite(f"a.png", frame)

vid.release()


# Calibration coordinates: pt1: (x: 75.4, y: 254, z: 0) pt2: (x: 75.4, y: 254+(255.5-32), z: 0)


# (rvec, tvec)= find_camera_transform_aruco(camera_matrix, distortion, cv2.imread("origin_calib.png"),
#     [[75.4], [254], [0]], [[75.4], [(254+(255.5-32))], [0]])

# print(rvec)
# print(tvec)

# np.save("camera_rotation.npy", rvec)
# np.save("camera_translation.npy", tvec)

camera_rotation = cv2.Rodrigues(np.load("camera_rotation.npy"))[0]
camera_translation = np.load("camera_translation.npy")


def find_x_y(u, v, z, camera_matrix, rotation_matrix, translation_vector):
  """Finds world X,Y coordinates given world Z and picture u,v coordinates.
  Arguments:
  u -- x coordinate of the pixel on the un-distorted image.
  v -- y coordinate of hte pixel on the un-distorted image.
  z -- world z coordinate. +z is closer to the camera
  Returns:
  (x, y) in world coordinates.
  """

  uv = np.array([[u], [v], [1]])
  inv_rotation_matrix = np.linalg.inv(rotation_matrix)
  inv_camera_matrix = np.linalg.inv(camera_matrix)
  lsm = np.dot(np.dot(inv_rotation_matrix, inv_camera_matrix), uv)
  rsm = np.dot(inv_rotation_matrix, translation_vector)

  s = (z + rsm[2, 0]) / lsm[2, 0]
  p = np.dot(inv_rotation_matrix, s *
             np.dot(inv_camera_matrix, uv) - translation_vector)
  return np.array([p[0], p[1]])


print(find_x_y(1108, 873, 0, camera_matrix, camera_rotation, camera_translation))
print(find_x_y(359, 874, 0, camera_matrix, camera_rotation, camera_translation))
print(find_x_y(337, 260, 0, camera_matrix, camera_rotation, camera_translation))
