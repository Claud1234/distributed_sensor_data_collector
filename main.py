#!/bin/env python3

import argparse
import os
import sys
import glob
import natsort
import cv2
import psycopg2

from src.process_frame import process_frame
from src.machine_learning import Detector

IMAGE_EXT = '.jpg'
RADAR_EXT = '_radar_*.json'

LABEL_FILE = 'assets/labels.txt'
MODEL_URL = 'https://tfhub.dev/tensorflow/ssd_mobilenet_v2/fpnlite_640x640/1'
MODEL_DIM = 640

DB = 'transport_ecosystem_management_db'
DB_USER = 'db_user'
DB_PASS = 'transport123'
DB_HOST = 'localhost'
DB_PORT = '5432'


def arg_parser() -> argparse:

    parser = argparse.ArgumentParser(description='Rosbag unpacker')
    parser.add_argument('-d', '--dir', help='Input directory to process')
    parser.add_argument('-t', '--thresh', default=0.45, help='Model detection threshold')
    parser.add_argument('-p', '--preview', action='store_true', help='Visualizes all processed frames before committing '
                                                                     'the data into the database. Useful for debugging.')
    parser.add_argument('-s', '--step', action='store_true', help='Allows stepping through the frames with space while visualizing')

    args = parser.parse_args()
    return args


def main(args: argparse) -> int:
    input_path = os.path.abspath(os.path.expanduser(args.dir))
    print("Processing", input_path)

    if not os.path.isdir(input_path):
        print('ERROR: Not a directory:', input_path, file=sys.stderr)
        return 1

    if not os.access(input_path, os.R_OK):
        print('ERROR: No read access to directory:', args.process_folder, file=sys.stderr)
        return 1

    # Read the class labels
    labels = dict()

    # Read class file
    with open(LABEL_FILE, 'r') as label_file:
        label_contents = label_file.readlines()

        for i, label in enumerate(label_contents):
            labels[i] = label.strip()

    # Machine learning
    detector = Detector(MODEL_URL, MODEL_DIM, args.thresh, labels)

    # Database
    db_conn = None
    if not args.preview:
        db_conn = psycopg2.connect(database=DB,
                                   user=DB_USER,
                                   password=DB_PASS,
                                   host=DB_HOST,
                                   port=DB_PORT)

        if db_conn:
            print('Connected to database')

    # Read all jpg files in the folder
    jpg_files = natsort.natsorted(glob.glob(os.path.join(input_path, f'*{IMAGE_EXT}')))

    file_count = len(jpg_files)

    for i, jpg in enumerate(jpg_files):
        no_ext = os.path.splitext(jpg)
        base_name = os.path.basename(no_ext[0])

        # Find radar files that mach the frame
        radar_file_regex = f'{no_ext[0]}{RADAR_EXT}'
        radar_files = natsort.natsorted(glob.glob(os.path.join(input_path, radar_file_regex)))

        percentage = round(i * 100 / file_count, 1)
        print(f'Processing: {base_name} ({percentage}%)')

        process_frame(args, jpg, radar_files, detector, db_conn)

        if args.preview:
            # ESC will close the program. Also without this line the 'X' button does not work.
            # OpenCV windows are fun...

            if args.step:
                if cv2.waitKey(0) == 27:
                    break
            else:
                if cv2.waitKey(1) == 27:
                    break

            # Quit when the user presses the 'X' button of the OpenCV video
            if cv2.getWindowProperty('preview', cv2.WND_PROP_VISIBLE) < 1:
                break

    if args.preview:
        cv2.destroyAllWindows()
    else:
        db_conn.close()

    return 0


if __name__ == '__main__':
    args = arg_parser()
    sys.exit(main(args))
