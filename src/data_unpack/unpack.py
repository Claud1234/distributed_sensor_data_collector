#!/usr/bin/env python3

import os
import sys
import time
import json
import shutil
import argparse
from pathlib import Path

from src.camera import Camera
from src.db import DBHandler
from src.data_process import BagParsing
# from src.data_process import LiveParsing


DEF_FPS = 10
DEF_CFG_PATH = 'config/data_unpack.json'


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


def topic_check(check_topics, db_topics):
    """
    Check and filter out topics which are not specified in database.

    Args:
    check_topics (str): List of the topics need to be checked.
    db_topics (str): List of the topics specified in database.

    Returns:
    valid_topics(str): topics pass the string checking.
    """
    valid_topics = dict()
    for sensor in db_topics.keys():
        tmp_topics = []
        for topic in db_topics[sensor]:
            if topic in check_topics:
                tmp_topics.append(topic)

        if len(tmp_topics) > 0:
            valid_topics[sensor] = tmp_topics
    return valid_topics


def get_output_folder(folder_name: str, output: str, overwrite: bool) -> list:
    """
    Creates an output folder for saving data
    Args:
        folder_name (str): The name of the folder.
        output (str): Path to the top level output directory
        overwrite (bool): Should the output directory be emptied (overwritten)
                          if it exists

    Raises:
        RuntimeError: When folder exists and overwrite is False

    Returns:
        list(str, str): (path to the output folder, rosbag name)
    """
    output_folder = os.path.join(output, folder_name)
    folder_exists = os.path.exists(output_folder)

    if folder_exists:
        if overwrite:
            shutil.rmtree(output_folder)
            folder_exists = False
        else:
            raise RuntimeError("Folder already exists! Use '-ow' to overwrite")

    if not folder_exists:
        os.makedirs(output_folder)

    return output_folder, folder_name


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

    output_path = args.directory if args.directory is not None \
        else cfg.get('data_path', '')
    output_path = os.path.expanduser(output_path)
    if output_path == '':
        print("No output folder specified!", file=sys.stderr)
        exit(1)

    # if args.mode == 'live':
    #   # TODO: need to check whether topics in cfg really available in pipeline
    #     print('live mode')
    #     live_topics = cfg.get('live_topics')
    #     valid_topics = topic_check(live_topics, db_topics)
    #     print(valid_topics)
    #     folder_name = time.strftime("%Y-%d-%m-%H-%M-%S", time.localtime())
    #     save_path, folder = get_output_folder(folder_name, output_path,
    #                                           args.overwrite)
    #     #LiveParsing()

    elif args.mode == 'bag':
        print('bag mode')
        if args.bag is None:
            raise RuntimeError('Please specify bag file for "bag unpack" mode!')
        else:
            bag_topics = BagParsing(args).read_topics()
            valid_topics = topic_check(bag_topics, db_topics)
            folder_name = Path(args.bag).stem
            save_path, folder = get_output_folder(folder_name, output_path,
                                                  args.overwrite)
            #print(save_path, folder)
            #print(valid_topics.get('radar')[0])

            #print(cfg.get('radar_2_topic'))
            #print(valid_topics)
            #print(bag_topics)
            BagParsing(args, cfg, db, save_path,
                        folder, valid_topics).parse_rosbag()
    else:
        exit()





    # raspi_cfg = cfg.get('raspi', dict())
    # camera = Camera(raspi_cfg['raspi_ground_height'],
    #                 raspi_cfg['raspi_angle'],
    #                 raspi_cfg['raspi_calib_intri'],
    #                 raspi_cfg['raspi_calibration_size'])
    # print(camera)
    # print(valid_topics)
    #DataProcess(args, cfg, db, save_path, folder, valid_topics).save_to_db()

    # Process rosbag
    # bag_parser = RosbagParser(args, db, save_path, folder,
    #                           valid_topics=valid_topics)
    # bag_parser.parse_rosbag(camera)

    db.close()


if __name__ == '__main__':
    # Parse arguments
    args = arg_parser()

    # Start the main program
    main(args)
