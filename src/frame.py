import os
import json

from src.topics import LastFrame
from src.serialize.image import RosImage
from src.serialize.pcl import PointCloud

def process_frame(frame: LastFrame, output_folder: str, radar_2d: bool, topics: dict) -> None:

    for sensor in frame.data:
        sensor_name = sensor[1:] if sensor[0] == '/' else sensor
        sensor_name = sensor_name.replace('/', '_')
        filename_base = f"f_{frame.frame_counter}_{sensor_name}"

        # Process camera images
        if sensor in topics.get("camera", []):

            # Create file paths
            img_name = filename_base + ".jpg"
            img_path = os.path.join(output_folder, img_name)

            # # Create and save image
            img = RosImage(frame.data[sensor])
            img.save(img_path)

        # Process radar pointclouds
        elif sensor in topics.get("radar", []):
            pcl = PointCloud(frame.data[sensor], frame.frame_counter, radar_2d)
            pcl.read_pc()
            pcl_obj = pcl.get_obj()

            radar_name = f'{filename_base}.json'
            radar_path = os.path.join(output_folder, radar_name)

            with open(radar_path, 'w') as radar_file:
                print(json.dumps(pcl_obj, indent=4, sort_keys=True), file=radar_file)

