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

sys.path.append('/home/devel/workspaces/finest_mobility/src/data_unpack/src')
import lidar_camera_projection

def get_lidar_xyz(i):
    with open(f'/home/devel/finest_data/lidar_cam_calib_db_project/lidar/seq_{i}_lidar.pkl', 'rb') as _fd:
        return pickle.load(_fd)[:,:3]


def get_radar_xyz(i, radar='radar1'):
    with open(f'/home/devel/finest_data/lidar_cam_calib_db_project/{radar}/seq_{i}_{radar}.json', 'rb') as _fd:
        df_radar1 = pd.concat([pd.DataFrame(x, index=[i]) for i,x in enumerate(json.load(_fd))])
        return df_radar1[['x','y','z']].values


def get_cam_img(i):
    return cv2.imread(f'/home/devel/finest_data/lidar_cam_calib_db_project/camera/seq_{i}_rgb.png',
                      cv2.IMREAD_UNCHANGED)


def get_sync_frames(df_sync, lidar_seq, radar='radar1'):
    '''Index is lidar'''
    cam_seq, radar_seq = df_sync.loc[lidar_seq][['cam',radar]].values
    return (get_lidar_xyz(lidar_seq),
            get_cam_img(cam_seq),
            get_radar_xyz(radar_seq, radar))


def get_sync_frames_v2(df_sync, radar_seq, radar='radar1'):
    '''Index is radar'''
    cam_seq, lidar_seq = df_sync.loc[radar_seq][['cam','lidar']].values
    return (get_lidar_xyz(lidar_seq),
            get_cam_img(cam_seq),
            get_radar_xyz(radar_seq, radar))


def get_neaerest_lidar(lidar_xyz, radar_xyz, tresh_m):
    return lidar_xyz[np.unique(np.argwhere(distance_matrix(lidar_xyz, radar_xyz) < tresh_m)[:,0])]


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


def get_lidar_cam_radar_sync(df_sync_radar1_lidar, df_lidar_cam_sync):
    '''Index is radar'''
    return  df_sync_radar1_lidar.set_index('radar1').join(df_lidar_cam_sync.set_index('lidar'), on='lidar')


def loop_step(i, df_lidar_cam_radar_sync, tr_radar_lidar, thresh_m=1.):
    lidar_xyz, img, radar_xyz = get_sync_frames_v2(df_lidar_cam_radar_sync, i, 'radar1')
    radar_xyz_tr = xyz_transform(radar_xyz, tr_radar_lidar)
    lidar_radar_xyz = get_neaerest_lidar(lidar_xyz, radar_xyz_tr, thresh_m)
    return lidar_xyz, radar_xyz, radar_xyz_tr, lidar_radar_xyz, img

def xyz_transform(np_xyz, np_mat_tr):
    assert np_mat_tr.shape == (3,4)
    assert np_xyz.shape[1] == 3
    return np.dot(np.asmatrix(np_mat_tr), np.vstack([np_xyz.T, np.ones(len(np_xyz))])).T

def main_loop(tmp_out, df_lidar_cam_radar_sync, radar1_cam_calib, tr_radar_lidar):
    if not os.path.exists(tmp_out):
        os.mkdir(tmp_out)
    for i in df_lidar_cam_radar_sync.index.values:
        f_base = os.path.join(tmp_out, f'lidar_cam_radar_{i}')
        f_img_out = f'{f_base}.png'
        f_pkl_out = f'{f_base}.pkl'
        f_plot_out = f'{f_base}.png'
        start = time.time()
        lidar_xyz, radar_xyz, radar_xyz_tr, lidar_radar_xyz, img = \
            loop_step(i, df_lidar_cam_radar_sync, tr_radar_lidar)

        # plot_lidar_radar(lidar_radar_xyz, radar_xyz, radar_xyz_tr, f_plot_out)
        # img_out = lidar_camera_projection.render_lidar_on_image(lidar_radar_xyz,
        #                                                         img,
        #                                                         lidar_cam_calib,
        #                                                         f_pkl_out)
        img_out = lidar_camera_projection.render_lidar_on_image(radar_xyz, img, radar1_cam_calib, f_pkl_out)

        cv2.imwrite(f_img_out, img_out)
        end = time.time()
        print('Average Time per frame of radar lidar clustering:', (end - start) * 1000)

df_sync_radar1_lidar =\
    pd.read_csv('/home/devel/finest_data/lidar_cam_calib_db_project/radar1-to-lidar-seq-sync.csv')

df_lidar_cam_sync = pd.read_csv('/home/devel/finest_data/lidar_cam_calib_db_project/lidar-to-cam-seq-sync.csv')

get_lidar_cam_radar_sync(df_sync_radar1_lidar, df_lidar_cam_sync).to_csv('/tmp/tmp.csv', index=False)

df_lidar_cam_radar_sync = pd.read_csv('/tmp/tmp.csv')

lidar_camera_calib =\
    lidar_camera_projection.read_calib_file('/home/devel/workspaces/finest_mobility/src/data_unpack/config/lidar_cam_calib.txt')

radar1_camera_calib =\
    lidar_camera_projection.read_calib_file('/home/devel/workspaces/finest_mobility/src/data_unpack/config/radar1_cam_calib_v2.txt')

tr_radar_lidar = np.asarray([[1.,0.,0.,0.3],
                             [0.,1.,0.,0.2],
                             [0.,0.,1.,-0.2]])

np_l, np_r, np_r_tr, np_lr, img= loop_step(30, df_lidar_cam_radar_sync,
                                       tr_radar_lidar)

plot_lidar_radar(lidar_xyz=None,
                 lidar_radar_xyz=np_lr,
                 radar_xyz=np_r,
                 radar_xyz_tr=np_r_tr)

main_loop('/home/devel/finest_data/out4',
          get_lidar_cam_radar_sync(df_sync_radar1_lidar, df_lidar_cam_sync),
          radar1_camera_calib, tr_radar_lidar)