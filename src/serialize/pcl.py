import struct
import numpy as np
import math
from src.camera import Camera, RadarPoint, Point3D

DATA_TYPES = {
    1: 'b', # INT8
    2: 'B', # UINT8
    3: 'h', # INT16
    4: 'H', # UINT16
    5: 'i', # INT32
    6: 'I', # UINT32
    7: 'f', # FLOAT32
    8: 'd'  # FLOAT64
}

class PointCloud:
    def __init__(self, ros_msg, frame_no):
        self.frame_no = frame_no

        self.height = ros_msg.height
        self.width = ros_msg.width
        self.fields = list(ros_msg.fields)
        self.point_step = ros_msg.point_step
        self.is_bigendian = ros_msg.is_bigendian
        self.data = ros_msg.data

        self.pcl_obj = {
            'frame_num': int(frame_no)
        }

        # TODO: This should not be hardcoded.
        self.camera = Camera((1920, 1080), -1.0, 3, 'src/calib.yaml', 0.036, False)

    def _get_format_str(self, format: int) -> str:

        if format not in DATA_TYPES.keys():
            raise ValueError(f"Error processing Pointcloud2: Unknown format: {format}")

        endian_char = '>' if self.is_bigendian else '<'
        type_char = DATA_TYPES[format]

        return endian_char + type_char

    def read_pc(self):
        points = []

        i = 0
        for _ in range(self.width * self.height):
            point_data = self.data[i:(i + self.point_step - 1)]
            i += self.point_step
            
            point = dict()

            for field in self.fields:
                format_str = self._get_format_str(field.datatype)

                ba = bytearray(point_data[field.offset:field.offset + 4])
                data = round(struct.unpack(format_str, ba)[0], 2)
                point[field.name] = data

            point['screen_coords'] = self._project_point(self.camera, point)

            if point['screen_coords'] is not None:
                points.append(point)
        
        self.pcl_obj['points'] = points


    def _project_point(self, camera, point):
        """
        Extracts all points in the cluster
        :param camera: instance of the camera object
        :param disable_cropping: If true, points will not be shifted to compensate for cropping
        :return: List containing (points, use_list)
        """

        coords3d = Point3D(np.array([point['y'], point['x'], point['z']]))

        coords3d = camera.transform_coordinates(coords3d)
        coords2d = camera.crit_infra_project_point(coords3d)

        # Only take the points which are in the visible area of the video
        # if camera.is_point_in_crop_box(coords2d):
        return coords2d.to_numpy().tolist()

        # else:
        #     return None

    def get_obj(self) -> dict:
        return self.pcl_obj