#!/bin/env python3

import argparse
import json
import os
import shutil
import sys

from pathlib import Path

from src.db import DBHandler
from src.rosbag_parser import RosbagParser

DEF_FPS = 10
DEF_CFG_PATH = './cfg.json'

def read_cfg(cfg_path: str) -> dict:

    items = ['db_user', 'db_pass', 'db_name', 
             'db_port', 'db_host']

    cfg = dict()

    try:
        with open(cfg_path, 'r') as cfg_file:
            cfg = json.load(cfg_file)

            for item in items:
                if item not in cfg.keys():
                    print(f"CFG Error! '{item}' not in configuration file!")
                    exit(1)
    except Exception as e:
        print(f"CFG Error! {str(e)}")
        exit(1)

    return cfg

def get_output_folder(bag_file: str, output: str, overwrite: bool) -> str:
    bag_name = Path(bag_file).stem
    output_folder = os.path.join(output, bag_name)
    folder_exists = os.path.exists(output_folder)

    if folder_exists:
        if overwrite:
            shutil.rmtree(output_folder)
            folder_exists = False
        else:
            print('Error! Folder already exists!')
            exit(1)

    if not folder_exists:
        os.makedirs(output_folder)
        print("Output directory created!")

    return output_folder, bag_name


def arg_parser():

    parser = argparse.ArgumentParser(description='Rosbag unpacker')
    parser.add_argument('-r', '--rosbag', required=True,
                        help='Rosbag to process')
    parser.add_argument('-o', '--output', 
                        help='Output directory to store the extracted frames')
    parser.add_argument('-f', '--fps', type=int,
                        default=DEF_FPS, help='Save data using this fps')
    parser.add_argument('-2d', '--radar-2d', action='store_true',
                        help='Add this flag if radar is using a 2D configuration')
    parser.add_argument('-ow', '--overwrite', action='store_true',
                        help='Overwrite the output directory')
    parser.add_argument('-p', '--progress',
                        action='store_true', help='Display progress bar')
    parser.add_argument('-l', '--list', action='store_true',
                        help='Lists topics in the rosbag and exits')
    parser.add_argument('-c', '--config', default=DEF_CFG_PATH, 
                        help='Path to configuration file')

    args = parser.parse_args()
    return args


def main(args):

    if not args.output and not args.list:
            print("No output folder specified!", file=sys.stderr)
            exit(1)

    # Read config file
    cfg_path = os.path.abspath(os.path.expanduser(args.config))
    print(f"Using CFG file: {cfg_path}")
    
    cfg = read_cfg(cfg_path)

    bag_parser = RosbagParser(args)
    bag_parser.read_topics()

    if args.list:
        bag_parser.print_topics()

    else:

        # Read sensor data from the
        db = DBHandler(cfg)
        
        db_topics = db.get_sensor_topics()

        # Read topics in rosbag
        bag_topics = bag_parser.get_topics()

        # Filter out topics, which are in rosbag and in the DB
        valid_topics = dict()

        for sensor in db_topics.keys():
            tmp_topics = []

            for topic in db_topics[sensor]:
                if topic in bag_topics:
                    tmp_topics.append(topic)

            if len(tmp_topics) > 0:
                valid_topics[sensor] = tmp_topics

        save_path, folder = get_output_folder(args.rosbag, args.output, args.overwrite)

        # Process rosbag
        bag_parser = RosbagParser(args, db, save_path, folder, valid_topics=valid_topics)
        bag_parser.parse_rosbag()


if __name__ == '__main__':
    args = arg_parser()
    main(args)
