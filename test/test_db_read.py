#!/bin/env python3

import argparse
import json
import os
import sys

import mysql.connector

from utils.visualize import visualize

DEF_CFG_PATH = './cfg.json'

class DBObject:
    def __init__(self, row, image_file, radar_files):
        self.obj_class = row[0]
        self.frame_id = row[1]
        self.speed = row[2]
        self.distance = row[3]
        self.bbox = row[4]
        self.self_speed = row[5]
        self.gps = row[6]
        self.timestamp = row[7]
        self.image_file = image_file
        self.radar_file = radar_files

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

def arg_parser() -> argparse:

    parser = argparse.ArgumentParser(description='Rosbag unpacker')
    group = parser.add_mutually_exclusive_group()
    group.add_argument('-w', '--where', help='Filter database results')
    group.add_argument('-f', '--frame', help='Search for a frame instead of an object')
    parser.add_argument('-l', '--limit', default=10, help='Number of results to receive')
    parser.add_argument('-c', '--config', default=DEF_CFG_PATH,
                           help='Path to configuration file'
                                ' (defaults to ./cfg.json)')

    args = parser.parse_args()
    return args

def main(args: argparse) -> int:

    # Read database info from config
    cfg_file = os.path.abspath(os.path.expanduser(args.config))
    print("Using config file:")
    print(cfg_file)

    db_cfg = read_cfg(cfg_file)

    # Database
    db_conn = None
    db_conn = mysql.connector.connect(
        host=db_cfg['db_host'],
        user=db_cfg['db_user'],
        passwd=db_cfg['db_pass'],
        port=db_cfg['db_port'],
        database=db_cfg['db_name']
    )

    if db_conn:
        print('Connected to database')
    else:
        return 1

    cur = db_conn.cursor()

    where_str = ''

    if args.where:
        where_str = f'WHERE {args.where}'
    elif args.frame:
        where_str = f'WHERE "FrameID" = {args.frame}'

    query = f'''SELECT "Class", "FrameID", "Speed", "Distance", "BoundingBox", "SelfSpeed", "GpsCoords", "Timestamp"
                FROM "DetectedObject"
                INNER JOIN "Frame"
                ON "DetectedObject"."FrameID" = "Frame"."ID"
                {where_str}
                LIMIT {args.limit};
            '''
    cur.execute(query)

    rows = cur.fetchall()


    print()
    print('Query returned:')
    db_objects = []
    for row in rows:
        img_file = None
        radar_files = []

        raw_query = f''' SELECT "SensorType", "DataFile"
                         FROM "SensorData"
                         WHERE "FrameID" = {row[1]};
                     '''
        cur.execute(raw_query)
        raw_rows = cur.fetchall()
        for raw in raw_rows:
            if raw[0] == "image":
                img_file = raw[1]
            elif raw[0] == "radar":
                radar_files.append(raw[1])
            else:
                print("Error! Unknown sensor type in database")

        db_object = DBObject(row, img_file, radar_files)
        print(f'Object class: {db_object.obj_class}')
        print(f'Frame ID: {db_object.frame_id}')
        print(f'Object speed: {db_object.speed}')
        print(f'Object distance: {db_object.distance}')
        print(f'Object bbox: {db_object.bbox}')
        print(f'Recorder speed: {db_object.self_speed}')
        print(f'Recorder GPS position: {db_object.gps}')
        print(f'Unix timestamp: {db_object.timestamp}')
        print()

        db_objects.append(db_object)

    db_conn.close()

    if args.frame or int(args.limit) == 1:
        visualize(db_objects)

    return 0

if __name__ == '__main__':
    args = arg_parser()
    sys.exit(main(args))