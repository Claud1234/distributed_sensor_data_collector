#!/bin/env python3

import argparse


import argparse
import rosbag

from serialization.rosbag_parser import RosbagParser

def arg_parser():

    parser = argparse.ArgumentParser(description='Rosbag unpacker')
    parser.add_argument('-r', '--rosbag', help='Rosbag to process')
    parser.add_argument('-o', '--output', help='Output directory to store the extracted frames')

    args = parser.parse_args()
    return args

def main(args):
    bag = rosbag.Bag(args.rosbag)

    RosbagParser(rosbag)

if __name__ == '__main__':
    args = arg_parser()
    main(args)
