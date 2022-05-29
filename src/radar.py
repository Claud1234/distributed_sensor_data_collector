import json
from dataclasses import dataclass


@dataclass
class RadarPoint:
    screen_coords: tuple
    velocity: float
    radar_id: int


def read_radar_points(json_files):
    points = []

    for i, json_file in enumerate(json_files):
        with open(json_file, 'r') as f:
            data = json.load(f)
            json_points = data.get('points', [])

        for json_point in json_points:
            point = RadarPoint(tuple(json_point.get('screen_coords', [0, 0])),
                            json_point.get('velocity', 0), i)

            points.append(point)

    return points


def is_point_in_box(box, point_coords):
    point_in_rect = box[0] <= point_coords[0] <= box[2] \
                    and box[1] <= point_coords[1] <= box[3]

    return point_in_rect


def get_detection_speeds(bboxes, radar_points):
    velocities = []

    for bbox in bboxes:
        box_speeds = []
        for radar_point in radar_points:
            if is_point_in_box(bbox, radar_point.screen_coords):
                box_speeds.append(abs(radar_point.velocity))

        if len(box_speeds) == 0:
            velocities.append(0)
        else:
            velocities.append(sum(box_speeds) / len(box_speeds))

    return velocities
