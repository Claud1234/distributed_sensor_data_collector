#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import numpy as np
import pandas as pd
import cv2
import json
import pickle
import os
import time
import matplotlib
from matplotlib import pylab as plt
import sys
from scipy.spatial import distance_matrix


def get_lidar_xyz(save_path, i):
    with open(os.path.join(save_path, 'lidar', f'seq_{i}_lidar.pkl'),
                'rb') as _fd:
        return pickle.load(_fd)[:, :3]


def get_radar_xyz(save_path, i, radar_id):
    with open(os.path.join(save_path, radar_id, f'seq_{i}_{radar_id}.json'),
                'rb') as _fd:
        df_radar = pd.concat(
            [pd.DataFrame(x, index=[i]) for i, x in enumerate(json.load(_fd))])
        return df_radar[['x', 'y', 'z']].values


def get_cam_img(save_path, i):
    return cv2.imread(os.path.join(save_path, 'camera', f'seq_{i}_rgb.png'),
                      cv2.IMREAD_UNCHANGED)


# def get_sync_frames(df_sync, lidar_seq, radar='radar1'):
#     '''Index is lidar'''
#     cam_seq, radar_seq = df_sync.loc[lidar_seq][['cam',radar]].values
#     return (get_lidar_xyz(lidar_seq),
#             get_cam_img(cam_seq),
#             get_radar_xyz(radar_seq, radar))


# def get_sync_frames_v2(df_sync, radar_seq, radar='radar1'):
#     '''Index is radar'''
#     cam_seq, lidar_seq = df_sync.loc[radar_seq][['cam','lidar']].values
#     return (get_lidar_xyz(lidar_seq),
#             get_cam_img(cam_seq),
#             get_radar_xyz(radar_seq, radar))


def plot_lidar_radar(lidar_radar_xyz, radar_xyz, radar_xyz_tr, figure_out=None, lidar_xyz=None):
    fig = plt.figure(figsize=(20, 20))
    ax = fig.add_subplot(projection='3d')
    if lidar_xyz is not None:
        ax.scatter(*lidar_xyz.T, color='yellow', alpha=.005, label='lidar_all')
    ax.scatter(*lidar_radar_xyz.T, color='blue', alpha=.2, label='LiDAR_clusters')
    ax.scatter(*radar_xyz.T, color='green', linewidths=3, label='radar_orig')
    ax.scatter(*radar_xyz_tr.T, color='red', linewidths=3, label='radar_transformed')
    ax.legend(loc=0)
    if figure_out is not None:
        plt.savefig(figure_out)
    plt.show()


def loop_step(radar_id, save_path, radar_seq, lidar_seq, cam_seq,
              tr_radar_lidar, thresh_m):
    lidar_xyz = get_lidar_xyz(save_path, lidar_seq)
    img = get_cam_img(save_path, cam_seq)
    radar_xyz = get_radar_xyz(save_path, radar_seq, radar_id)
    radar_xyz_tr = xyz_transform(radar_xyz, tr_radar_lidar)
    lidar_radar_xyz = get_neaerest_lidar(lidar_xyz, radar_xyz_tr, thresh_m)
    return lidar_xyz, radar_xyz, radar_xyz_tr, lidar_radar_xyz, img


def xyz_transform(np_xyz, np_mat_tr):
    assert np_mat_tr.shape == (3, 4)
    assert np_xyz.shape[1] == 3
    return np.dot(np.asmatrix(np_mat_tr),
                    np.vstack([np_xyz.T, np.ones(len(np_xyz))])).T


def get_neaerest_lidar(lidar_xyz, radar_xyz, tresh_m):
    return lidar_xyz[np.unique(np.argwhere(
                distance_matrix(lidar_xyz, radar_xyz) < tresh_m)[:, 0])]
                