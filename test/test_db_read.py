#!/bin/env python3

import sys
import argparse
import psycopg2
from dataclasses import dataclass

DB = 'transport_ecosystem_management_db'
DB_USER = 'db_user'
DB_PASS = 'transport123'
DB_HOST = 'localhost'
DB_PORT = '5432'

from utils.visualize import visualize

class DBObject:
    def __init__(self, row):
        self.obj_class = row[0]
        self.frame_id = row[1]
        self.speed = row[2]
        self.distance = row[3]
        self.bbox = row[4]
        self.self_speed = row[5]
        self.gps = row[6]
        self.timestamp = row[7]
        self.image_file = row[8]
        self.radar_file = row[9]


def arg_parser() -> argparse:

    parser = argparse.ArgumentParser(description='Rosbag unpacker')
    group = parser.add_mutually_exclusive_group()
    group.add_argument('-w', '--where', help='Filter database results')
    group.add_argument('-f', '--frame', help='Search for a frame instead of an object')
    parser.add_argument('-l', '--limit', default=10, help='Number of results to receive')

    args = parser.parse_args()
    return args

def main(args: argparse) -> int:
    # Database
    db_conn = None
    db_conn = psycopg2.connect(database=DB,
                                user=DB_USER,
                                password=DB_PASS,
                                host=DB_HOST,
                                port=DB_PORT)

    if db_conn:
        print('Connected to database')
    else:
        return 1

    cur = db_conn.cursor()

    where_str = ''

    if args.where:
        where_str = f'WHERE {args.where}'
    elif args.frame:
        where_str = f'WHERE "frame_id" = {args.frame}'

    query = f'''SELECT "Class", "frame_id", "Speed", "Distance", "Bounding_Box", "Self_Speed", "GPS_Coords", "Timestamp", "image_file", "radar_point_cloud" 
                FROM "Detected_Objects"
                INNER JOIN "Frame"
                ON "Detected_Objects"."frame_id" = "Frame"."Id"
                INNER JOIN "Sensor_Data"
                ON "Frame"."Sensor_Data" = "Sensor_Data"."Id"
                {where_str}
                LIMIT {args.limit};
            '''
    cur.execute(query)

    rows = cur.fetchall()

    print()
    print('Query returned:')
    db_objects = []
    for row in rows:
        db_object = DBObject(row)
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