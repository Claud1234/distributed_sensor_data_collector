import argparse
import cv2
import numpy as np
import os

from src.ml_algorithms.ml_base import DetectorBase
from src.radar import RadarPoint, read_radar_points, get_detection_speeds
from src.debug.visualize import visualize
from src.database_connector import add_to_db


def process_frame(args: argparse, data_path: str, files: dict, detector: DetectorBase, 
                  output_video: np.ndarray) -> int:  

    if len(files.get('camera', [])) == 0:
        print("Warning! Frame with no camera image")

    # TODO: Currently only process the image from camera 0
    image_path = os.path.join(data_path, files.get('camera', [])[0])

    image = cv2.imread(image_path)
    # image = cv2.resize(image, (1280, 720))


    height = image.shape[0]
    width = image.shape[1]
    
    radar_files = [os.path.join(data_path, radar) for radar in files.get('radar', [])]

    _, boxes, scores, labels = detector.detect(image)
    # print(boxes)

    bboxes = []
    for box in boxes:
        bbox = (int(box[1] * width),
                int(box[0] * height),
                int(box[3] * width),
                int(box[2] * height))

        bboxes.append(bbox)


    radar_points = read_radar_points(radar_files)

    velocities = get_detection_speeds(bboxes, radar_points)
    threshold = detector.get_threshold()

    if args.preview or args.video:
        visualize(image, labels, scores, bboxes, threshold, radar_points, velocities, output_video)

    # else:
    #     if True in (scores > threshold):
    #         add_to_db(db_conn, detector, image_file, radar_files, labels, bboxes, threshold, velocities, scores)

    return 0
