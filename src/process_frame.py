import argparse
import cv2

from src.machine_learning import Detector
from src.radar import RadarPoint, read_radar_points, get_detection_speeds
from src.debug.visualize import visualize
from src.database_connector import add_to_db


def process_frame(args: argparse, image_file: str, radar_file: str, detector: Detector, db_conn) -> int:
    image = cv2.imread(image_file)

    height = image.shape[0]
    width = image.shape[1]

    _, boxes, scores, labels = detector.detect(image)

    bboxes = []
    for box in boxes:
        bbox = (int(box[1] * width),
                int(box[0] * height),
                int(box[3] * width),
                int(box[2] * height))

        bboxes.append(bbox)

    radar_points = read_radar_points(radar_file)

    velocities = get_detection_speeds(bboxes, radar_points)

    threshold = detector.get_threshold()

    if args.preview:
        visualize(image, labels, scores, bboxes, threshold, radar_points, velocities)

    else:
        if True in (scores > threshold):
            add_to_db(db_conn, image_file, radar_file, labels, bboxes, threshold, velocities, scores)

    return 0
