#!/bin/env python3

import argparse
from sqlite3 import Time

import rospy
import rosbag
import yaml
from PIL import Image
import io
import os
from pathlib import Path
import shutil

from src.rosbag_parser import RosbagParser

DEF_FPS = 10

def save_frame(args, last_messages, time_ns):
    # print(last_messages.values())
    global output_folder
    if not None in last_messages.values():
        im = Image.open(io.BytesIO(last_messages['image']))

        filename = f"frame_{last_messages['frame_no']}.jpg"
        filename = os.path.join(output_folder, filename)
        print(filename)

        im.save(filename)

def arg_parser():

    parser = argparse.ArgumentParser(description='Rosbag unpacker')
    parser.add_argument('-r', '--rosbag', help='Rosbag to process')
    parser.add_argument('-o', '--output', help='Output directory to store the extracted frames')
    parser.add_argument('-f', '--fps', type=int, default=DEF_FPS, help='Save data using this fps')
    parser.add_argument('-2d', '--radar-2d', action='store_true', help='Add this flag if radar is using a 2D configuration')
    parser.add_argument('-ow', '--overwrite', action='store_true', help='Overwrite the output directory')
    parser.add_argument('-p', '--progress', action='store_true', help='Display progress bar')

    args = parser.parse_args()
    return args

def main(args):

    # Process rosbag
    bag_parser = RosbagParser(args)
    bag_parser.parse_rosbag()

if __name__ == '__main__':
    args = arg_parser()
    main(args)
