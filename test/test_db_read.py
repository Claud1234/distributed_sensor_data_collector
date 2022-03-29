#!/bin/env python3

import sys
import argparse
import psycopg2

DB = 'transport_ecosystem_management_db'
DB_USER = 'db_user'
DB_PASS = 'transport123'
DB_HOST = 'localhost'
DB_PORT = '5432'

def arg_parser() -> argparse:

    parser = argparse.ArgumentParser(description='Rosbag unpacker')
    parser.add_argument('-w', '--where', help='filter database results')
    parser.add_argument('-l', '--limit', default=10, help='number of results to receive')

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

    where_str = f'WHERE {args.where}' if args.where else ''

    query = f'''SELECT "Class", "Speed", "Distance", "Bounding_Box", "Self_Speed", "GPS_Coords", "Timestamp", "image_file", "radar_point_cloud" 
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
    for row in rows:
        obj_class, speed, distance, bbox, self_speed, gps, timestamp, image_file, radar_file = row
        print(f'Object class: {obj_class}')
        print(f'Object speed: {speed}')
        print(f'Object distance: {distance}')
        print(f'Object bbox: {bbox}')
        print(f'Recorder speed: {self_speed}')
        print(f'Recorder GPS position: {gps}')
        print(f'Unix timestamp: {timestamp}')
        print()

    db_conn.close()
    return 0

if __name__ == '__main__':
    args = arg_parser()
    sys.exit(main(args))