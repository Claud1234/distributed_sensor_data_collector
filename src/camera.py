"""
Contains functions which are needed for point projection and image undistoration
"""
import sys

import yaml
import numpy as np
import cv2
from math import sin, cos, radians
from dataclasses import dataclass

CAMERA_OFFSET = 0


@dataclass
class CropRect:
    x_min: int
    y_min: int
    x_max: int
    y_max: int


@dataclass
class Point3D:
    x: int
    y: int
    z: int

    def __init__(self, array: np.array):
        if len(array) != 3:
            raise ValueError(f"Cannot convert from array of size {len(array)} to Point3D!")

        self.x, self.y, self.z = array

    def to_numpy(self) -> np.array:
        return np.array([self.x, self.y, self.z])

@dataclass
class Point2D:
    x: int
    y: int

    def __init__(self, array: np.array):
        if len(array) != 2:
            raise ValueError(f"Cannot convert from array of size {len(array)} to Point2D!")

        self.x, self.y = array

    def to_numpy(self) -> np.array:
        return np.array([self.x, self.y])

    def to_tuple(self) -> tuple:
        return self.x, self.y


@dataclass
class RadarPoint:
    coords_3d: Point3D
    screen_coors: Point2D
    velocity: float
    distance: float

class Camera:

    def __init__(self, dim: tuple, camera_altitude: float, device_angle: float,
                 calib_file: str, checkerboard_square_size, cuda: bool = False) -> None:
        """
        Class containing functions for camera calibration
        Parameters
        ----------
        dim: tuple containing the size of the frame
        camera_altitude: height of the device from ground during mounting (in meters)
        device_angle: Mounting angle of the device, in degrees
        calib_file: Path to the YAML file containing the calibration data
        cuda: If True, undistortion matrices will be loaded into CUDA memory
        checkerboard_square_size: Square size of the checkerboard that was used for camera calibration
        """

        self.dim = dim

        # Camera matrix
        self.K = None

        # Distortion vector
        self.D = None

        # Projection matrix
        self.P = None

        # Read calibration data
        self._get_calib_data(calib_file)

        # Size of a checkerboard's square
        self.checkerboard_square_size = checkerboard_square_size

        # Altitude of the camera
        self.camera_altitude = camera_altitude - CAMERA_OFFSET  # RAISES camera by one meter for better radar mapping

        # Translation matrix. Set it to zeroes, we will do point translation manually
        self.T = np.array([0.0, 0.0, 0.0])

        # Change coordinate system to camera's coordinates
        self.translate_mat = np.array([0,  # X
                                       0,  # Y
                                       self.camera_altitude])  # Z

        # Matrix to convert units from square size to meters
        self.scale_mat = np.array([-self.checkerboard_square_size,  # X
                                   self.checkerboard_square_size,  # Y
                                   self.checkerboard_square_size])  # Z

        # Initialize undistort
        self.new_K, roi = cv2.getOptimalNewCameraMatrix(self.K, self.D, self.dim, 1, self.dim)

        # Generate undistort maps
        self.undistort_mapx, self.undistort_mapy = cv2.initUndistortRectifyMap(self.K, self.D, None,
                                                                               self.new_K, self.dim, 5)

        # Cropping rectangle
        self.crop_rect = self._calculate_crop_rect_from_roi(roi)

        self.undistort_mapx_gpu = None
        self.undistort_mapy_gpu = None

        # Load the undistortion matrices to GPU memory
        if cuda:
            self.undistort_mapx_gpu = cv2.cuda_GpuMat(self.undistort_mapx)
            self.undistort_mapy_gpu = cv2.cuda_GpuMat(self.undistort_mapy)

        # Rotation of radar points
        x_rot_angle = 90 + device_angle
        y_rot_angle = 0
        z_rot_angle = 0

        # Rotational matrices of each coordinate
        xr = np.array([[1.0, 0.0, 0.0],
                       [0.0, cos(radians(x_rot_angle)), -sin(radians(x_rot_angle))],
                       [0.0, sin(radians(x_rot_angle)), cos(radians(x_rot_angle))]])

        yr = np.array([[cos(radians(y_rot_angle)), -sin(radians(y_rot_angle)), 0.0],
                       [sin(radians(y_rot_angle)), cos(radians(y_rot_angle)), 0.0],
                       [0.0, 0.0, 1.0]])

        zr = np.array([[cos(radians(z_rot_angle)), 0.0, sin(radians(z_rot_angle))],
                       [0.0, 1.0, 0.0],
                       [-sin(radians(z_rot_angle)), 0.0, cos(radians(z_rot_angle))]])

        # Final rotational matrix
        self.R = np.dot(np.dot(zr, yr), xr)
        # self.R = self.quaternion_rotation_matrix(-0.5, 0.4999999999999999, -0.5, 0.5000000000000001)

    def _calculate_crop_rect_from_roi(self, roi):
        """
        Calculates a cropping rectangle from ROI
        Parameters
        ----------
        roi: Region of interest, as returned by getOptimalNewCameraMatrix

        Returns
        -------
        Croprect describing the ROI
        """
        x, y, w, h = roi
        return CropRect(x, y, x + w, y + h)

    def _get_calib_data(self, calib_file: str) -> None:
        """
        Reads calibration data from ROS formatted YAML file
        Parameters
        ----------
        calib_file YAML file created by ROS, which contains the calibration data

        Returns
        -------
        None
        """

        with open(calib_file, 'r') as file:
            calib_data = yaml.safe_load(file)

            # Camera matrix
            rows = calib_data['camera_matrix']['rows']
            cols = calib_data['camera_matrix']['cols']
            self.K = np.array(calib_data['camera_matrix']['data']).reshape((rows, cols))

            # Distortion vector
            rows = calib_data['distortion_coefficients']['rows']
            cols = calib_data['distortion_coefficients']['cols']
            self.D = np.array(calib_data['distortion_coefficients']['data']).reshape((rows, cols))

            # Projection matrix
            rows = calib_data['projection_matrix']['rows']
            cols = calib_data['projection_matrix']['cols']
            self.P = np.array(calib_data['projection_matrix']['data']).reshape((rows, cols))

        if self.K is None or self.D is None or self.P is None:
            raise RuntimeError("Reading calibration data failed!")

    def render_crop_rect(self, frame: np.array) -> np.array:
        """
        Draws a cropped rectangle on the uncropped frame.
        Useful for debugging
        Parameters
        ----------
        frame: The OpenCV frame to draw on

        Returns
        -------
        OpenCV frame with crop rectangle drawn on it
        """

        return cv2.rectangle(frame, (self.crop_rect.x_min, self.crop_rect.y_min),
                             (self.crop_rect.x_max, self.crop_rect.y_max), (0, 0, 255), 3)

    def update_crop_rect(self, new_crop_rect: CropRect) -> None:
        """
        Updates the cropping rectangle used for cropping
        Parameters
        ----------
        new_crop_rect: New cropping rectangle, format of [(x0, y0), (y0, y1)]

        Returns
        -------
        None
        """

        self.crop_rect = new_crop_rect

    def undistort_frame(self, frame: np.array) -> np.array:
        """
        Undistorts a frame from a wide angle camera
        Parameters
        ----------
        frame: Frame to undistort

        Returns
        -------
        Undistored frame
        """

        undistorted_frame = cv2.remap(frame, self.undistort_mapx, self.undistort_mapy, cv2.INTER_LINEAR)

        return undistorted_frame

    def undistort_frame_gpu(self, frame: cv2.cuda_GpuMat) -> cv2.cuda_GpuMat:
        """
        Undistorts a frame from a wide angle camera using GPU (CUDA)
        Parameters
        ----------
        frame: CUDA frame to undistort

        Returns
        -------
        Undistored CUDA frame
        """

        if self.undistort_mapx_gpu is None or self.undistort_mapy_gpu is None:
            raise RuntimeError('CUDA not initialized for camera!')

        undistorted_frame = cv2.cuda.remap(frame, self.undistort_mapx_gpu, self.undistort_mapy_gpu, cv2.INTER_LINEAR)
        return undistorted_frame

    def quaternion_rotation_matrix(self, q0, q1, q2, q3):
        """
        Covert a quaternion into a full three-dimensional rotation matrix.
    
        Input
        :param Q: A 4 element array representing the quaternion (q0,q1,q2,q3) 
    
        Output
        :return: A 3x3 element matrix representing the full 3D rotation matrix. 
                This rotation matrix converts a point in the local reference 
                frame to a point in the global reference frame.
        """
        # Extract the values from Q
        # q0 = Q[0]
        # q1 = Q[1]
        # q2 = Q[2]
        # q3 = Q[3]
        
        # First row of the rotation matrix
        r00 = 2 * (q0 * q0 + q1 * q1) - 1
        r01 = 2 * (q1 * q2 - q0 * q3)
        r02 = 2 * (q1 * q3 + q0 * q2)
        
        # Second row of the rotation matrix
        r10 = 2 * (q1 * q2 + q0 * q3)
        r11 = 2 * (q0 * q0 + q2 * q2) - 1
        r12 = 2 * (q2 * q3 - q0 * q1)
        
        # Third row of the rotation matrix
        r20 = 2 * (q1 * q3 - q0 * q2)
        r21 = 2 * (q2 * q3 + q0 * q1)
        r22 = 2 * (q0 * q0 + q3 * q3) - 1
        
        # 3x3 rotation matrix
        rot_matrix = np.array([[r00, r01, r02],
                            [r10, r11, r12],
                            [r20, r21, r22]])
        return rot_matrix

    def crit_infra_project_point(self, point: Point3D) -> Point2D:
        """
        Wrapper around OpenCV's projectPoints function that works out of the box
        with critical infrastructure sensor
        Parameters
        ----------
        point: Points to be projected on the camera image

        Returns
        -------
        Projected points
        """

        point = np.dot(self.R, point.to_numpy())
        mat = np.array([[point[0]], [point[1]], [point[2]], [1.0]])
        dst = np.dot(self.P, mat)

        x = dst[0, 0]
        y = dst[1, 0]
        w = dst[2, 0]

        if w != 0:
            coords = (int(x / w), int(y / w))
        else:
            coords = (0, 0)
        return Point2D(coords)

    def crit_infra_project_lines(self, lines: list) -> list:
        """
        Wrapper around OpenCV's projectPoints function that works out of the box for lines
        with critical infrastructure sensor
        Parameters
        ----------
        lines: Lines to project

        Returns
        -------
        Projected points
        """

        projected_lines = []

        for line in lines:
            imgpts, _ = cv2.projectPoints(line, self.R, self.T, self.new_K, self.new_D)
            projected_lines.append(imgpts)
        return projected_lines

    def draw_grid(self, img: np.array, x_min: int, y_min: int, x_max: int, y_max: int,
                  x_step: int, y_step: int, disable_cropping: bool) -> np.array:
        """
        Projects a grid on the image with selected properties
        Parameters
        ----------
        img: Image to project the grid on
        x_min: Minimum x coordinate of the grid (in meters, relative to camera)
        y_min: Minimum y coordinate of the grid (in meters, relative to camera)
        x_max: Maximum x coordinate of the grid (in meters, relative to camera)
        y_max: Maximum y coordinate of the grid (in meters, relative to camera)
        x_step: Distance of grid lines horizontally (in meters)
        y_step: Distance of grid lines vertically (in meters)
        disable_cropping: Set to True if image is not cropped after undistort

        Returns
        -------

        """
        # Draw X grid
        x_coords = np.arange(x_min, x_max + 1, x_step)
        for x in x_coords:
            p1_coords = self.transform_coordinates(Point3D(np.array([x, y_min, 0])))
            p1 = self.crit_infra_project_point(p1_coords)

            p2_coords = self.transform_coordinates(Point3D(np.array([x, y_max, 0])))
            p2 = self.crit_infra_project_point(p2_coords)

            if not disable_cropping:
                p1 = self.calculate_new_point_for_cropping(p1)
                p2 = self.calculate_new_point_for_cropping(p2)

            img = cv2.line(img, p1.to_tuple(), p2.to_tuple(), (0, 255, 255), thickness=3)

        # Draw Y grid
        y_coords = np.arange(y_min, y_max + 1, y_step)
        for y in y_coords:
            p1_coords = self.transform_coordinates(Point3D(np.array([x_min, y, 0])))
            p1 = self.crit_infra_project_point(p1_coords)

            p2_coords = self.transform_coordinates(Point3D(np.array([x_max, y, 0])))
            p2 = self.crit_infra_project_point(p2_coords)

            if not disable_cropping:
                p1 = self.calculate_new_point_for_cropping(p1)
                p2 = self.calculate_new_point_for_cropping(p2)

            img = cv2.line(img, p1.to_tuple(), p2.to_tuple(), (0, 255, 255), thickness=3)

        return img

    def get_cropped_video_dimensions(self) -> np.array:
        """
        Returns the dimensions of the video after cropping. Does NOT perform the cropping, just return the size
        Returns
        -------
        A tuple in the format (x, y), where x and y are the size of the cropped video in pixels
        """

        return self.crop_rect.x_max - self.crop_rect.x_min, self.crop_rect.y_max - self.crop_rect.y_min

    def is_point_in_crop_box(self, point: Point2D) -> bool:
        """
        Checks if a point on a screen is inside the crop box. (Will be visible after cropping the frame)
        Parameters
        ----------
        point: Two dimensional point on the screen

        Returns
        -------
        True if point is inside the box, false otherwise
        """

        return self.crop_rect.x_min <= point.x < self.crop_rect.x_max \
               and self.crop_rect.y_min <= point.y < self.crop_rect.y_max

    def calculate_new_point_for_cropping(self, points: np.array) -> np.array:
        """
        Compensates points' coordinates for image cropping by subtracting the coordinates of the upper left corner
        of the cropping rectangle from the points' coordinates
        Parameters
        ----------
        points: Points with coordinates on the image

        Returns
        -------
        Point with compensated coordinates
        """

        cropped_points = []

        if len(points) == 0:
            return points
        else:
            for point in points:
                point.screen_coors.x -= self.crop_rect.x_min
                point.screen_coors.y -= self.crop_rect.y_min

                cropped_points.append(point)

        return np.array(cropped_points)

    def crop_frame(self, frame: np.array) -> np.array:
        """
        Crops the frame according to the crop rectangle
        Parameters
        ----------
        frame: The frame that is going to be cropped

        Returns
        -------
        Cropped frame
        """
        return frame[self.crop_rect.y_min:self.crop_rect.y_max, self.crop_rect.x_min:self.crop_rect.x_max]

    def crop_frame_gpu(self, frame: cv2.cuda_GpuMat) -> cv2.cuda_GpuMat:
        """
        Crops the frame according to the crop rectangle on CUDA
        Parameters
        ----------
        frame: The CUDA frame that is going to be cropped

        Returns
        -------
        Cropped CUDA frame
        """

        xmin = self.crop_rect.x_min
        ymin = self.crop_rect.y_min
        xmax = self.crop_rect.x_max - self.crop_rect.x_min
        ymax = self.crop_rect.y_max - self.crop_rect.y_min

        return cv2.cuda_GpuMat(frame, (xmin, ymin, xmax, ymax))

    def transform_coordinates(self, point_in_m: Point3D) -> Point3D:
        """
        Transforms 3D point coordinates in meters to camera coordinates
        Parameters
        ----------
        point_in_m: Tuple containing the 3D point coordinate in meters

        Returns
        -------
        3D position of the point in camera coordinates
        """

        return Point3D((point_in_m.to_numpy() - self.translate_mat) / self.scale_mat)
