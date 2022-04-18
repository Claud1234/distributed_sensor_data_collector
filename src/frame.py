import os
import json

from src.topics import TopicData, Topics, LastFrame
from src.serialize.image import RosImage
from src.serialize.pcl import PointCloud

def process_frame(frame: LastFrame, output_folder: str) -> None:

    # Create filenames
    filename_base = f"frame_{frame.frame_counter}"

    img_name = filename_base + ".jpg"
    img_path = os.path.join(output_folder, img_name)

    radar_name = filename_base + "_radar.json"
    radar_path = os.path.join(output_folder, radar_name)

    # Create and save image
    img = RosImage(frame.image)
    img.save(img_path)

    # Process the pointcloud
    pcl = PointCloud(frame.radar_pc, frame.frame_counter)
    pcl.read_pc()
    pcl_obj = pcl.get_obj()

    with open(radar_path, 'w') as radar_file:
        print(json.dumps(pcl_obj, indent=4, sort_keys=True), file=radar_file)

