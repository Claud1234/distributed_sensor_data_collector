import argparse
import os

import cv2
import numpy as np

from src.db import DBHandler
from src.debug.visualize import visualize
from src.ml_algorithms.ml_base import DetectorBase
from src.radar import get_detection_speeds, read_radar_points


def process_frame(args: argparse, first_frame: bool, frame_id: int, data_path: str, db_conn: DBHandler, 
                  model_data: dict, files: dict, detector: DetectorBase, output_video: np.ndarray) -> int:  

    if len(files.get('camera', [])) == 0:
        print("Warning! Frame with no camera image")

    # TODO: Currently only process the image from camera 0
    image_path = os.path.join(data_path, files.get('camera', [])[0])

    image = cv2.imread(image_path)

    height = image.shape[0]
    width = image.shape[1]
    
    radar_files = [os.path.join(data_path, radar) for radar in files.get('radar', [])]

    ids, boxes, scores, labels = detector.detect(image)

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
        visualize(image, first_frame, labels, scores, bboxes, threshold, radar_points, velocities, output_video)

    else:
        if True in (scores > threshold):
            db_conn.save_objs(frame_id, model_data, threshold, ids, labels, 
                              bboxes, velocities, scores)

        db_conn.mark_frame_processed(frame_id, model_data.get('id'))

    return 0
