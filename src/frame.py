import os
import json

from src.topics import LastFrame
from src.serialize.image import RosImage
from src.serialize.pcl import PointCloud

def process_frame(frame: LastFrame, save_path: str, folder: str, 
                  radar_2d: bool, topics: dict) -> list:

    frame_sensors = list()

    for sensor in frame.data:

        unknown_sensor = False

        sensor_name = sensor[1:] if sensor[0] == '/' else sensor
        sensor_name = sensor_name.replace('/', '_')
        filename_base = f"f_{frame.frame_counter}_{sensor_name}"

        # TODO: Not used, placeholder for sensors that save data directly to the DB
        sensor_data = None

        # Path to file with raw data, if exists
        sensor_file = None

        # Process camera images
        if sensor in topics.get("camera", []):

            # Create file paths
            sensor_file = filename_base + ".jpg"
            sensor_file_path = os.path.join(save_path, sensor_file)

            # # Create and save image
            img = RosImage(frame.data[sensor])
            img.save(sensor_file_path)

        # Process radar pointclouds
        elif sensor in topics.get("radar", []):
            pcl = PointCloud(frame.data[sensor], frame.frame_counter, radar_2d)
            pcl.read_pc()
            pcl_obj = pcl.get_obj()

            sensor_file = f'{filename_base}.json'
            sensor_file_path = os.path.join(save_path, sensor_file)

            with open(sensor_file_path, 'w') as radar_file:
                print(json.dumps(pcl_obj, indent=4, sort_keys=True), file=radar_file)

        else:
            unknown_sensor = True

        if not unknown_sensor:
            # Store sensor data to the structure
            frame_sensor = {
                'topic': sensor,
                'file': os.path.join(folder, sensor_file),
                'data': sensor_data
            }

            frame_sensors.append(frame_sensor)

    return frame_sensors
