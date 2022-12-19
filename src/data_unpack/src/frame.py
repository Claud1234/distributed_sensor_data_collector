import os
import cv2
import json
import pickle
import numpy as np

from src.topics import LastFrame
from src.serialize.image import RosImage
from src.serialize.pcl import PointCloud
from src.camera import Camera
from src.lidar_camera_projection import *


def process_frame(cfg, camera_r_matrix: Camera, frame: LastFrame,
                  save_path: str, folder: str, radar_2d: bool,
                  topics: dict) -> list:

    frame_index = f"f_{frame.frame_counter}"
    rgb = frame_index + "_rgb" + ".png"
    rgb_path = os.path.join(save_path, rgb)
    rgb_bytes = frame.data[cfg.get('image_topic')]
    rgb_array = cv2.imdecode(np.frombuffer(rgb_bytes, np.uint8), -1)
    cv2.imwrite(rgb_path, rgb_array)
    rgb_db_info = {
        'topic': cfg.get('image_topic'),
        'file': rgb_path,
        'data': None
    }

    lidar = frame_index + "_lidar" + ".pkl"
    lidar_path = os.path.join(save_path, lidar)
    with open(lidar_path, 'wb') as f:
        pickle.dump(frame.data[cfg.get('image_topic')], f)
    lidar_db_info = {
        'topic': cfg.get('lidar_topic'),
        'file': lidar_path,
        'data': None
    }

    radar_1 = frame_index + "_radar1" + ".json"
    radar_1_path = os.path.join(save_path, radar_1)
    pcl_1 = PointCloud(camera_r_matrix, frame.data[cfg.get("radar_1_topic")],
                       frame.frame_counter, radar_2d)
    pcl_1.read_pc()
    pcl_1_obj = pcl_1.get_obj()
    with open(radar_1_path, 'w') as f:
        print(json.dumps(pcl_1_obj, indent=4, sort_keys=True), file=f)
    radar_1_db_info = {
        'topic': cfg.get('radar_1_topic'),
        'file': radar_1_path,
        'data': None
    }

    radar_2 = frame_index + "_radar2" + ".json"
    radar_2_path = os.path.join(save_path, radar_2)
    pcl_2 = PointCloud(camera_r_matrix, frame.data[cfg.get("radar_2_topic")],
                       frame.frame_counter, radar_2d)
    pcl_2.read_pc()
    pcl_2_obj = pcl_1.get_obj()
    with open(radar_2_path, 'w') as f:
        print(json.dumps(pcl_2_obj, indent=4, sort_keys=True), file=f)
    radar_2_db_info = {
        'topic': cfg.get('radar_2_topic'),
        'file': radar_2_path,
        'data': None
    }


    projected_point_pkl = frame_index + "_cam_view_points" + ".pkl"
    projected_point_pkl_path = os.path.join(save_path, projected_point_pkl)
    projected_img = frame_index + "_proj_img" + ".png"
    projected_img_path = os.path.join(save_path, projected_img)
    lidar_projection(rgb_array, frame.data[cfg.get('lidar_topic')],
                     projected_point_pkl_path, projected_img_path)
    projected_point_db_info = {
        'topic': None,
        'file': projected_point_pkl_path,
        'data': None
    }
    projected_img_db_info = {
        'topic': None,
        'file': projected_img_path,
        'data': None
    }

    frame_db_info = [rgb_db_info, lidar_db_info, radar_1_db_info,
                     radar_2_db_info, projected_point_db_info,
                     projected_img_db_info]


    # for sensor in frame.data:
    #     # /image_color_rect/compressed
    #     # /ti_mmwave/radar_scan_pcl_1
    #     # /velodyne_packets
    #
    #
    #     unknown_sensor = False
    #
    #     # i.e. /image_color_rect/compressed
    #     sensor_name = sensor[1:] if sensor[0] == '/' else sensor
    #     # i.e _image_color_rect_compressed
    #     sensor_name = sensor_name.replace('/', '_')
    #     # i.e f_0_image_color_rect_compressed
    #     indexed_name = f"f_{frame.frame_counter}_{sensor_name}"
    #
    #     # TODO: Not used, placeholder for sensors that save data directly to DB
    #     sensor_data = None
    #
    #     # Path to file with raw data, if exists
    #     sensor_file = None
    #
    #     # Process camera images
    #     #if sensor in topics.get("camera", []):
    #
    #     # Create file paths
    #
    #
    #     print("aaaaaa:", sensor)
    #     img_bytes = frame.data[topics.get("camera")]
    #     img_array = cv2.imdecode(np.frombuffer(img_bytes, np.uint8), -1)
    #     cv2.imwrite(img_file_path, img_array)
    #
    #         # # Create and save image
    #         # img = RosImage(frame.data[sensor])
    #         # img.save(sensor_file_path)
    #
    #     # Process radar pointclouds
    #     #elif sensor in topics.get("radar", []):
    #         # The projection of the radar to camera.
    #     print(topics.get("radar"))
    #     exit()
    #     pcl = PointCloud(camera_r_matrix, frame.data[topics.get("radar")], frame.frame_counter, radar_2d)
    #     pcl.read_pc()
    #     pcl_obj = pcl.get_obj()
    #
    #     sensor_file = f'{filename_base}.json'
    #     sensor_file_path = os.path.join(save_path, sensor_file)
    #
    #     with open(sensor_file_path, 'w') as radar_file:
    #         print(json.dumps(pcl_obj, indent=4, sort_keys=True), file=radar_file)
    #
    #     if sensor in topics.get('lidar', []):
    #         sensor_file = f'{filename_base}.pkl'
    #         sensor_file_path = os.path.join(save_path, sensor_file)
    #         print(img_array)
    #         point_lidar = frame.data[sensor]
    #         # lidar_projection(img_array, point_lidar, filename_base, save_path)
    #
    #
    #
    #         with open(sensor_file_path, 'wb') as lidar_file:
    #             pickle.dump(point_lidar, lidar_file)
    #
    #
    #     else:
    #         unknown_sensor = True
    #
    #     if not unknown_sensor:
    #         # Store sensor data to the structure


    return frame_db_info
