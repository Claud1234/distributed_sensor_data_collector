import numpy as np
import pandas as pd
import cv2
import json
import pickle
import os
import matplotlib
from matplotlib import pylab as plt
import sys
from scipy.spatial import distance_matrix

import lidar_camera_projection


class RLCSyncSession(object):

    DEFAULT_RADAR_NR = 'radar1'

    __slots__ = [
        'lidar_dir_path',
        'radar_dir_path',
        'camera_dir_path',
        'radar_nr'
    ]

    def __init__(self, **kwargs):
        for k in self.__slots__:
            if k not in kwargs.keys():
                if k == 'radar_nr':
                    setattr(self, 'radar_nr', self.DEFAULT_RADAR_NR)
                else:
                    raise ValueError(f'Argument not set: {k}')
            else:
                setattr(self, k, kwargs[k])

    def __dict__(self):
        return {k: getattr(self, k) for k in self.__slots__}

    




def get_lidar_xyz(i, lidar_dir_path):
    with open(os.path.join(lidar_dir_path, f'seq_{i}_lidar.pkl'), 'rb') as _fd:
        return pickle.load(_fd)[:, :3]


def get_radar_xyz(i, radar_dir_path, radar='radar1'):
    with open(os.path.join(radar_dir_path,
                           f'seq_{i}_{radar}.json'), 'rb') as _fd:
        df_radar1 =\
            pd.concat(
                [pd.DataFrame(x,
                              index=[i]) for i, x in enumerate(json.load(_fd))])
        return df_radar1[['x', 'y', 'z']].values


def get_cam_img(i, cam_dir_path):
    return cv2.imread(os.path.join(cam_dir_path, f'seq_{i}_rgb.png'),
                      cv2.IMREAD_UNCHANGED)

def get_sync_frames(df_sync, radar_seq, radar='radar1'):
    """Index is radar"""
    cam_seq, lidar_seq = df_sync.loc[radar_seq][['cam', 'lidar']].values
    return (get_lidar_xyz(lidar_seq),
            get_cam_img(cam_seq),
            get_radar_xyz(radar_seq, radar))

