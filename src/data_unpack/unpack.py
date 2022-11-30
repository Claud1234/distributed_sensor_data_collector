#!/bin/env python3

import argparse
import json
import os
import shutil
import sys
from pathlib import Path

from src.camera import Camera
from src.db import DBHandler
from src.rosbag_parser import RosbagParser

DEF_FPS = 10
DEF_CFG_PATH = 'config/data_unpack.json'

# Parameters for camera-radar fusion
# TODO: This should not be hardcoded.
CAMERA_GROUND_HEIGHT = -1.0
CAMERA_ANGLE = 3
CAMERA_CALIBRATION_SIZE = 0.036
CAMERA_CALIB_DATA = 'src/calib.yaml'


def read_cfg(cfg_path: str) -> dict:
    """
    Reads a JSON formatted config file

    Args:
        cfg_path (str): Path to the config file to read

    Returns:
        dict: Dictionary object representing the contents
              of the configuration file
    """

    items = ['db_user', 'db_pass', 'db_name',
             'db_port', 'db_host']

    with open(cfg_path, 'r') as cfg_file:
        cfg = json.load(cfg_file)

        for item in items:
            if item not in cfg.get('db', {}).keys():
                raise RuntimeError(f"'{item}' not in configuration file!")
    return cfg


def get_output_folder(bag_file: str, output: str, overwrite: bool) -> list:
    """
    Creates an output folder for saving data

    Args:
        bag_file (str): Path to the bag file that is being processed
        output (str): Path to the top level output directory
        overwrite (bool): Should the output directory be emptied (overwritten)
                          if it exists

    Raises:
        RuntimeError: When folder exists and overwrite is False

    Returns:
        list(str, str): (path to the output folder, rosbag name)
    """
    bag_name = Path(bag_file).stem
    output_folder = os.path.join(output, bag_name)
    folder_exists = os.path.exists(output_folder)

    if folder_exists:
        if overwrite:
            shutil.rmtree(output_folder)
            folder_exists = False
        else:
            raise RuntimeError("Folder already exists!")

    if not folder_exists:
        os.makedirs(output_folder)

    return output_folder, bag_name


def arg_parser() -> argparse.Namespace:
    """
    Parses program command line arguments

    Returns:
        argparse.Namespace: Parsed arguments
    """

    parser = argparse.ArgumentParser(description='Data unpacker')
    parser.add_argument('-m', '--mode', required=True, choices=['bag', 'live'],
                        help='Choose mode to process bag files or live topics')
    parser.add_argument('-b', '--bag', default=None,
                        help='The bag file path')
    parser.add_argument('-d', '--directory',
                        help='Directory to store the extracted frames. '
                        'Overrides the value in the configuration file.')
    parser.add_argument('-f', '--fps', type=int,
                        default=DEF_FPS, help='Save data using this fps')
    parser.add_argument('-cf', '--calib-file',
                        help="Path to camera calibration file to use for "
                             "sensor fusion. "
                             "The supported format is ROS' YAML format.")
    parser.add_argument('-2d', '--radar-2d', action='store_true',
                        help='Add this flag if radar is using 2D configuration')
    parser.add_argument('-ow', '--overwrite', action='store_true',
                        help='Overwrite the output directory')
    parser.add_argument('-p', '--progress',
                        action='store_true', help='Display progress bar')
    # parser.add_argument('-l', '--list', action='store_true',
    #                     help='Lists topics in the rosbag and exits')
    parser.add_argument('-c', '--config', default=DEF_CFG_PATH,
                        help='Path to configuration file')
    args = parser.parse_args()
    return args


def main(args: argparse.Namespace):
    """
    Main function

    Args:
        args argparse.Namespace: Arguments parsed by argparse
    """

    # Read config file
    cfg_path = os.path.abspath(os.path.expanduser(args.config))
    print(f"Using config file: {cfg_path}")
    try:
        cfg = read_cfg(cfg_path)
    except Exception as e:
        print(f"CFG Error! {str(e)}")
        exit(1)

    # Read the topics in database, specified in 'sensor'.
    db = DBHandler(cfg.get('db', dict()))
    db_topics = db.get_sensor_topics()
    # print(db_topics['lidar'])

    topics_check = []
    if args.mode == 'live':
        print('live mode')
        live_topics = cfg.get('live_topics')
        topics_check = live_topics
    elif args.mode == 'bag':
        print('bag mode')
        if args.bag is None:
            raise RuntimeError('Please specify bag file for "bag unpack" mode!')
        else:
            bag_parser = RosbagParser(args)
            bag_parser.read_topics()
            bag_parser.print_topics()

            bag_topics = bag_parser.get_topics()
            topics_check = bag_topics

    # Filter out topics, which are in rosbag and in the DB
    valid_topics = dict()
    for sensor in db_topics.keys():
        tmp_topics = []
        for topic in db_topics[sensor]:
            if topic in topics_check:
                tmp_topics.append(topic)

        if len(tmp_topics) > 0:
            valid_topics[sensor] = tmp_topics

    output_path = args.directory if args.directory is not None \
                            else cfg.get('data_path', '')
    output_path = os.path.expanduser(output_path)
    if output_path == '':
        print("No output folder specified!", file=sys.stderr)
        exit(1)

    # TODO: Here need to check the 'live' or 'bag' mode. 'bag' save as bag name,
    # TODO: 'live' save as current wall time
    try:
        save_path, folder = get_output_folder(args.rosbag, output_path,
                                              args.overwrite)
    except Exception as e:
        print(f"Error creating output folder: {str(e)}")
        exit(1)
    print("Output folder created")
        #
        # # TODO: This should not be hardcoded.
        # camera = Camera(CAMERA_GROUND_HEIGHT, CAMERA_ANGLE,
        #                 CAMERA_CALIB_DATA, CAMERA_CALIBRATION_SIZE)
        #
        # # Process rosbag
        # bag_parser = RosbagParser(args, db, save_path, folder,
        #                           valid_topics=valid_topics)
        # bag_parser.parse_rosbag(camera)

    db.close()


if __name__ == '__main__':
    # Parse arguments
    args = arg_parser()

    # Start the main program
    main(args)
