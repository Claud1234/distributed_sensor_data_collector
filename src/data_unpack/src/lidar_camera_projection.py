#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Script to project lidar points to rgb images and blank images,
create lidar_image that background is rgb images and blank-black images.
Single thread process, slow
"""
import cv2
import pickle
import numpy as np
import matplotlib.pyplot as plt


CALIB_FILE_PATH = 'config/lidar_cam_calib.txt'


def lidar_projection(img_array, pts_lidar, pkl_path, png_path):
    """
    :param img_array: Array of the rgb image
    :param pts_lidar: Array of the all LiDAR points
    :param pkl_path: Path for saving cam-view points as pickle file
    :param png_path: Path for saving the point-projected-rgb image.
    :return:
    """

    calib = read_calib_file(CALIB_FILE_PATH)

    rgb = cv2.cvtColor(img_array, cv2.COLOR_BGR2RGB)
    pts_lidar = pts_lidar[:, :3]

    rgb_proj = render_lidar_on_image(pts_lidar, rgb, calib, pkl_path)
    plt.imsave(png_path, rgb_proj)


def render_lidar_on_image(pts_lidar, rgb, calib, pkl_path):
    img_h, img_w, img_c = rgb.shape

    # projection matrix (project from velo2cam2)
    proj_velo2cam2 = project_velo_to_cam2(calib)

    # apply projection
    pts_2d = project_to_image(pts_lidar.transpose(), proj_velo2cam2)

    # Filter lidar points to be within image FOV
    inds = np.where((pts_2d[0, :] < img_w) & (pts_2d[0, :] >= 0) &
                    (pts_2d[1, :] < img_h) & (pts_2d[1, :] >= 0) &
                    (pts_lidar[:, 0] > 0)
                    )[0]

    # Dump the cam-view-points to a pickle file
    front_view_labels = np.zeros(len(pts_2d.T))
    front_view_labels[inds] = 1
    waymo_struct = {}
    waymo_struct['camera_coordinates'] = \
        np.vstack([front_view_labels, pts_2d]).T.astype(np.uint16)
    waymo_struct['3d_points'] = pts_lidar[:, :3]
    with open(pkl_path, 'wb') as _fd:
        pickle.dump(waymo_struct, _fd)

    # Filter out pixels points
    imgfov_pc_pixel = pts_2d[:, inds]

    # Retrieve depth from lidar
    imgfov_pc_lidar = pts_lidar[inds, :]
    imgfov_pc_lidar = np.hstack((imgfov_pc_lidar,
                                 np.ones((imgfov_pc_lidar.shape[0], 1))))
    imgfov_pc_cam2 = proj_velo2cam2 @ imgfov_pc_lidar.transpose()

    cmap = plt.cm.get_cmap('hsv', 2048)
    cmap = np.array([cmap(i) for i in range(2048)])[:, :3] * 2047

    for i in range(imgfov_pc_pixel.shape[1]):
        #depth = imgfov_pc_cam2[2, i]
        #color = cmap[int(2048.0 / depth), :]
        color = (0,255,0)
        cv2.circle(rgb, (int(np.round(imgfov_pc_pixel[0, i])),
                         int(np.round(imgfov_pc_pixel[1, i]))),
                   2, color=tuple(color), thickness=2)
    return rgb


def project_velo_to_cam2(calib):
    P_velo2cam_ref = np.vstack((calib['Tr_velo_to_cam'].reshape(3, 4),
                                np.array([0., 0., 0., 1.])))  # velo2ref_cam
    R_ref2rect = np.eye(4)
    R0_rect = calib['R0_rect'].reshape(3, 3)  # ref_cam2rect
    R_ref2rect[:3, :3] = R0_rect
    P_rect2cam2 = calib['P2'].reshape((3, 4))
    proj_mat = P_rect2cam2 @ R_ref2rect @ P_velo2cam_ref
    return proj_mat


def load_lidar_bin(bin_filename):
    pts = np.fromfile(bin_filename, dtype=np.float32)
    pts = pts.reshape((-1, 4))
    return pts


def read_calib_file(filepath):
    """
    Read in a calibration file and parse into a dictionary.
    Ref: https://github.com/utiasSTARS/pykitti/blob/master/pykitti/utils.py
    """
    data = {}
    with open(filepath, 'r') as f:
        for line in f.readlines():
            line = line.rstrip()
            if len(line) == 0:
                continue
            key, value = line.split(':', 1)
            # The only non-float values in these files are dates, which
            # we don't care about anyway
            try:
                data[key] = np.array([float(x) for x in value.split()])
            except ValueError:
                pass
    return data


def project_to_image(points, proj_mat):
    """
    Apply the perspective projection
    Args:
        points:     3D points in camera coordinate [3, npoints]
        proj_mat:   Projection matrix [3, 4]
    """
    num_pts = points.shape[1]

    # Change to homogeneous coordinate
    points = np.vstack((points, np.ones((1, num_pts))))
    points = proj_mat @ points
    points[:2, :] /= points[2, :]
    return points[:2, :]

