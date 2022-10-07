#!/bin/env python3

import argparse
import json
import os
import sys

import cv2

from src.db import DBHandler
from src.ml_algorithms.mobilenet2_640 import Mobilenet_640_detector
from src.ml_algorithms.yolo_escooter import YOLOeScooterDetect
from src.process_frame import process_frame

ML_MODEL_DICT = {
    'mobilenetv2_640_coco': Mobilenet_640_detector,
    'YOLOeScooterDetect': YOLOeScooterDetect
}

IMAGE_EXT = '.jpg'
RADAR_EXT = '_radar_*.json'

DEF_CFG_PATH = './cfg.json'


def arg_parser() -> argparse:

    parser = argparse.ArgumentParser(description='Object Detector')
    parser.add_argument('-d', '--data', 
                        help='Directory to store the extracted frames. '
                        'Overrides the value in the configuration file.')
    parser.add_argument('-m', '--model', 
                        help='Model to be used for detection. Required.')
    parser.add_argument('-lm', '--list-models', action='store_true', 
                        help='Lists valid models and exits')
    parser.add_argument('-t', '--thresh', default=None, 
                        help='Model detection threshold')
    parser.add_argument('-p', '--preview', action='store_true', 
                        help='Visualizes all processed frames before committing '
                        'the data into the database. Useful for debugging.')
    parser.add_argument('-s', '--step', action='store_true', 
                        help='Allows stepping through the frames with space while visualizing '
                        '(only used when --preview option is specified')
    parser.add_argument('-v', '--video', 
                        help='Save output as video (cannot be used together with the --preview option)')
    parser.add_argument('-c', '--config', default=DEF_CFG_PATH,
                        help='Path to configuration file (defaults to ./cfg.json)')

    args = parser.parse_args()
    return args

def get_valid_models(models: dict) -> list:
    valid_models = []

    for model in models.keys():
        if model in ML_MODEL_DICT.keys():
            valid_models.append(model)

    return valid_models

def print_models(models: dict, valid_models: list) -> None:
    print('Valid models are:')

    for name, model in models.items():
        if name in valid_models:
            print(f"* {model['name']}")
            print(f"\t Comment:  {model['comment']}")
            print(f"\t ML model: {model['model_name']}")
            print(f"\t Dataset:  {model['dataset_name']}")

def read_cfg(cfg_path: str) -> dict:

    items = ['db_user', 'db_pass', 'db_name',
             'db_port', 'db_host']

    cfg = dict()

    try:
        with open(cfg_path, 'r') as cfg_file:
            cfg = json.load(cfg_file)

            for item in items:
                if item not in cfg.get('db', {}).keys():
                    print(f"CFG Error! '{item}' not in configuration file!")
                    exit(1)
    except Exception as e:
        print(f"CFG Error! {str(e)}")
        exit(1)

    return cfg

def main(args: argparse) -> int:
    print()
    # Read database info from config

    cfg_file = os.path.abspath(os.path.expanduser(args.config))
    print("Using config file:")
    print(cfg_file)

    cfg = read_cfg(cfg_file)

    # Connect to the database
    db_conn = DBHandler(cfg.get('db', {}))
    db_conn.fetch_models()
    models = db_conn.get_models()

    valid_models = get_valid_models(models)
    
    if len(valid_models) == 0:
        print("Error! No models found that are implemented and exist in the DB!")
        exit(1)

    # If valid model not specified, print the list of models and exit
    if args.list_models or not args.model or \
       args.model not in valid_models:
       
        msgstr = "\nError: valid model not specified!" if not args.list_models else "\n"
        print(msgstr)
        print_models(models, valid_models)
        return 0

    input_path = args.data if args.data is not None else cfg.get('data_path', '')
    input_path = os.path.abspath(os.path.expanduser(input_path))

    if input_path == '' and not args.list:
        print("No data folder specified in config or in arguments!", file=sys.stderr)
        exit(1)

    print("Processing", input_path)

    if not os.path.isdir(input_path):
        print('ERROR: Not a directory:', input_path, file=sys.stderr)
        return 1

    if not os.access(input_path, os.R_OK):
        print('ERROR: No read access to directory:', args.process_folder, file=sys.stderr)
        return 1

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
    print(f"\tThreshold cutoff: {detector.get_threshold()}\n")

    # Get the list of unprocessed frames
    print("Getting unprocessed frames from the database")
    frames = db_conn.get_unprocessed_frames(models[args.model]['id'])
    
    if len(frames) == 0:
        print("No frames found!")
        exit(0)

    output_video = None
    # if args.video:
    #     print(f"Saving output to {args.video}")
        
    #     # Read first image to get the size of the images
    #     image = cv2.imread(jpg_files[0])

    #     fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    #     output_video = cv2.VideoWriter(args.video, fourcc, 10, (image.shape[1], image.shape[0]))


    for i, frame_id in enumerate(frames):

        percentage = round(i * 100 / len(frames), 1)
        print(f'Processing: ({percentage}%)')

        files = db_conn.fetch_sensor_data(frame_id)
        process_frame(args, input_path, files, detector, output_video)

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

    # if args.video:
    #     output_video.release()

    db_conn.close()

    return 0


if __name__ == '__main__':
    args = arg_parser()
    sys.exit(main(args))
