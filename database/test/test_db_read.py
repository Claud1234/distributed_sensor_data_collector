#!/bin/env python3

import argparse
import json
import os
import sys

import mysql.connector
import os

from utils.visualize import visualize

DEF_CFG_PATH = '../../src/data_unpack/config/data_unpack.json'

def parse_multipoint(mp_str: str) -> tuple:
    output = []
    coords_str = mp_str.split('MULTIPOINT((')[1][:-2]
    point_array = coords_str.split('),(')
    for point in point_array:
        split_point = point.split()
        point_int = []
        for coord in split_point:
            point_int.append(int(coord))
        
        output.append(tuple(point_int))
    return tuple(output)


class DBObject:
    def __init__(self, row, image_file, radar_files,file_path):
        self.class_name = row[0]
        self.frame_id = row[1]
        self.speed = row[2]
        self.distance = row[3]
        self.bounding_box = parse_multipoint(row[4])
        self.confidence = row[5]
        self.detection_model = row[6]
        self.timestamp = row[7]

        self.image_file = os.path.join(file_path, image_file)
        self.radar_files = [os.path.join(file_path, radar_file) for radar_file in radar_files]

def read_cfg(cfg_path: str) -> dict:
    """
    Reads a JSON formatted config file

    Args:
        cfg_path (str): Path to the config file to read

    Returns:
        dict: Dictionary object representing the contents
              of the configuration file
    """
    cfg = {}

    items = ['db_user', 'db_pass', 'db_name',
             'db_port', 'db_host']

    with open(cfg_path, 'r') as cfg_file:
        cfg = json.load(cfg_file)

        for item in items:
            if item not in cfg.get('db', {}).keys():
                raise RuntimeError(f"'{item}' not in configuration file!")

    return cfg

def arg_parser() -> argparse:

    parser = argparse.ArgumentParser(description='Rosbag unpacker')
    group = parser.add_mutually_exclusive_group()
    group.add_argument('-w', '--where', 
                       help='Filter database results')
    group.add_argument('-f', '--frame', 
                       help='Search for a frame instead of an object')
    parser.add_argument('-l', '--limit', default=10,
                        help='Number of results to receive (default: 10)')
    parser.add_argument('-c', '--config', default=DEF_CFG_PATH,
                        help='Path to configuration file (defaults to ./data_unpack.json)')
    parser.add_argument('-d', '--dir',
                        help='Path to raw data (image and radar files)')


    args = parser.parse_args()
    return args

def main(args: argparse) -> int:

    # Read database info from config
    if not args.dir:
        print("Please specify the directory where files are stored using the --dir flag")
        exit(1)

    file_path = os.path.abspath(os.path.expanduser(args.dir))

    cfg_file = os.path.abspath(os.path.expanduser(args.config))
    print("Using config file:")
    print(cfg_file)

    cfg = read_cfg(cfg_file)
    db_cfg = cfg.get('db')

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
        where_str = f'WHERE frame.id = {args.frame}'

    query = f'''SELECT object_class.name, frame.id, object.speed, object.distance, 
                       ST_AsText(object.bounding_box), object.confidence, detection_model.name, 
                       frame.timestamp
                FROM object
                INNER JOIN frame
                ON object.frame_id = frame.id
                INNER JOIN object_class
                ON object.class_id = object_class.id
                INNER JOIN detection_model
                ON object.detection_model_id = detection_model.id
                {where_str}
                LIMIT {args.limit};
            '''

    print("Query was:")
    print(query)

    cur.execute(query)

    rows = cur.fetchall()

    if len(rows) == 0:
        print("No results.")
        exit(0)

    print()
    print('Query returned:')
    db_objects = []
    for row in rows:
        img_file = None
        radar_files = []

        raw_query = f''' SELECT sensor_type.type_name, frame_sensor.file_name
                         FROM frame_sensor
                         INNER JOIN sensor
                         ON frame_sensor.sensor_id = sensor.id
                         INNER JOIN sensor_type
                         ON sensor.sensor_type_id = sensor_type.id
                         WHERE frame_sensor.frame_id = {row[1]};
                     '''
        cur.execute(raw_query)
        raw_rows = cur.fetchall()
        for raw in raw_rows:
            if raw[0] == "camera":
                img_file = raw[1]
            elif raw[0] == "radar":
                radar_files.append(raw[1])

        db_object = DBObject(row, img_file, radar_files, file_path)
        print(f'class_name: {db_object.class_name}')
        print(f'frame_id: {db_object.frame_id}')
        print(f'speed: {db_object.speed}')
        print(f'distance: {db_object.distance}')
        print(f'bounding_box: {db_object.bounding_box}')
        print(f'confidence: {db_object.confidence}')
        print(f'detection_model: {db_object.detection_model}')
        print(f'timestamp: {db_object.timestamp}')

        print()

        db_objects.append(db_object)

    db_conn.close()

    if args.frame or int(args.limit) == 1 or len(rows) == 1:
        visualize(db_objects)

    return 0

if __name__ == '__main__':
    args = arg_parser()
    sys.exit(main(args))
