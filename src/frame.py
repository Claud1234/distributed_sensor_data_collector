import os
import json

from src.topics import TopicData, Topics, LastFrame
from src.serialize.image import RosImage
from src.serialize.pcl import PointCloud

def process_frame(frame: LastFrame, output_folder: str, radar_2d: bool) -> None:

    # Create filenames
    filename_base = f"frame_{frame.frame_counter}"

    img_name = filename_base + ".jpg"
    img_path = os.path.join(output_folder, img_name)


    # Create and save image
    # TODO: Currently only saving the first image is supported
    img = RosImage(frame.images[list(frame.images.keys())[0]])
    img.save(img_path)

    # Process the pointclouds
    
    radar_pcs = sorted(frame.radar_pc.keys())

    for i, radar_pc in enumerate(radar_pcs):
        pcl = PointCloud(frame.radar_pc[radar_pc], frame.frame_counter, radar_2d)
        pcl.read_pc()
        pcl_obj = pcl.get_obj()

        radar_name = f'{filename_base}_radar_{i}.json'
        radar_path = os.path.join(output_folder, radar_name)

        with open(radar_path, 'w') as radar_file:
            print(json.dumps(pcl_obj, indent=4, sort_keys=True), file=radar_file)

