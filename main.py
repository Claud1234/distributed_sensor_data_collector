#!/bin/env python3

import argparse
import os
import sys
import glob
import natsort
import cv2
import psycopg2

from src.process_frame import process_frame
from src.ml_algorithms.mobilenet2_640 import Mobilenet_640_detector
from src.ml_algorithms.yolo_escooter import YOLOeScooterDetect

ML_MODEL_DICT = {
    'mobilenetv2_640': Mobilenet_640_detector,
    'YOLOeScooterDetect': YOLOeScooterDetect
}

IMAGE_EXT = '.jpg'
RADAR_EXT = '_radar_*.json'

DB = 'transport_ecosystem_management_db'
DB_USER = 'db_user'
DB_PASS = 'transport123'
DB_HOST = 'localhost'
DB_PORT = '5432'


def arg_parser() -> argparse:

    parser = argparse.ArgumentParser(description='Rosbag unpacker')
    parser.add_argument('-d', '--dir', help='Input directory to process')
    parser.add_argument('-m', '--model', help='Model to be used for detection. Required.')
    parser.add_argument('-lm', '--list-models', action='store_true', help='Lists valid models and exits')
    parser.add_argument('-t', '--thresh', default=0.45, help='Model detection threshold')
    parser.add_argument('-p', '--preview', action='store_true', help='Visualizes all processed frames before committing '
                                                                     'the data into the database. Useful for debugging.')
    parser.add_argument('-s', '--step', action='store_true', help='Allows stepping through the frames with space while visualizing (only used when --preview option is specified')
    parser.add_argument('-v', '--video', help='Save output as video (cannot be used together with the --preview option)')

    args = parser.parse_args()
    return args

def print_models() -> None:
    print('Valid models are:')
    for model in ML_MODEL_DICT.keys():
        print(f"\t* {model}")

def main(args: argparse) -> int:
    input_path = os.path.abspath(os.path.expanduser(args.dir))
    print("Processing", input_path)

    if not os.path.isdir(input_path):
        print('ERROR: Not a directory:', input_path, file=sys.stderr)
        return 1

    if not os.access(input_path, os.R_OK):
        print('ERROR: No read access to directory:', args.process_folder, file=sys.stderr)
        return 1

    # If valid model not specified, print the list of models and exit
    if args.list_models or not args.model or args.model not in ML_MODEL_DICT.keys():
        msgstr = "\nError: valid model not specified!" if not args.list_models else "\n"
        print(msgstr)
        print_models()
        return 0

    # Output video flag
    if args.preview and args.video:
        print("--preview and --video flags cannot be used together!")
        return 1

    # Machine learning
    detector = ML_MODEL_DICT[args.model](args.thresh)

    print(f"\nUsing model: {args.model}")
    print(f"\tModel Name: {detector.get_name()}")
    print(f"\tModel Dataset: {detector.get_dataset()}")
    print(f"\tInput Image Dimensions: {detector.get_dim()}x{detector.get_dim()}")
    print(f"\tThreshold cutoff: {args.thresh}\n")

    # Database
    db_conn = None
    if not args.preview and not args.video:
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

    output_video = None
    if args.video:
        print(f"Saving output to {args.video}")
        
        # Read first image to get the size of the images
        image = cv2.imread(jpg_files[0])

        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        output_video = cv2.VideoWriter(args.video, fourcc, 10, (image.shape[1], image.shape[0]))

    for i, jpg in enumerate(jpg_files):
        no_ext = os.path.splitext(jpg)
        base_name = os.path.basename(no_ext[0])

        # Find radar files that mach the frame
        radar_file_regex = f'{no_ext[0]}{RADAR_EXT}'
        radar_files = natsort.natsorted(glob.glob(os.path.join(input_path, radar_file_regex)))

        percentage = round(i * 100 / file_count, 1)
        print(f'Processing: {base_name} ({percentage}%)')

        process_frame(args, jpg, radar_files, detector, db_conn, output_video)

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

    if args.video:
        output_video.release()

    if not args.preview and not args.video:
        db_conn.close()

    return 0


if __name__ == '__main__':
    args = arg_parser()
    sys.exit(main(args))
