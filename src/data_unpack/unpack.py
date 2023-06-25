#!/usr/bin/env python3

import os
import cv2
import pickle
import sys
import time
import json
import shutil
import argparse
import progressbar
import numpy as np
import pandas as pd
from pathlib import Path
from multiprocessing import Pool

from src.camera import Camera
from src.db import DBHandler
# from src.data_process import BagParsing
from src.data_process_multi_core import BagParsing
from src.lidar_camera_projection import lidar_projection, read_calib_file, \
                                        render_lidar_on_image
# from src.data_process import LiveParsing

from src.radar_lidar_cam_fusion import loop_step

DEF_FPS = 10
DEF_CFG_PATH = 'config/data_unpack.json'


def read_cfg(cfg_path: str) -> dict:
    """
    Reads a JSON formatted config file

    Args:
        cfg_path (str): Path to the config file to read

    Returns:
        dict: Dictionary object representing the contents
              of the configuration file
    """

    items = ['db_user', 'db_pass', 'db_name',
             'db_port', 'db_host']

    with open(cfg_path, 'r') as cfg_file:
        cfg = json.load(cfg_file)

        for item in items:
            if item not in cfg.get('db', {}).keys():
                raise RuntimeError(f"'{item}' not in configuration file!")
    return cfg


def topic_check(check_topics, db_topics):
    """
    Check and filter out topics which are not specified in database.

    Args:
    check_topics (str): List of the topics need to be checked.
    db_topics (str): List of the topics specified in database.

    Returns:
    valid_topics(str): topics pass the string checking.
    """
    valid_topics = dict()
    for sensor in db_topics.keys():
        tmp_topics = []
        for topic in db_topics[sensor]:
            if topic in check_topics:
                tmp_topics.append(topic)

        if len(tmp_topics) > 0:
            valid_topics[sensor] = tmp_topics
    return valid_topics


def get_output_folder(folder_name: str, output: str, overwrite: bool) -> list:
    """
    Creates an output folder for saving data
    Args:
        folder_name (str): The name of the folder.
        output (str): Path to the top level output directory
        overwrite (bool): Should the output directory be emptied (overwritten)
                          if it exists

    Raises:
        RuntimeError: When folder exists and overwrite is False

    Returns:
        list(str, str): (path to the output folder, rosbag name)
    """
    output_folder = os.path.join(output, folder_name)
    folder_exists = os.path.exists(output_folder)

    # if folder_exists:
    #     if overwrite:
    #         shutil.rmtree(output_folder)
    #         folder_exists = False
    #     else:
    #         raise RuntimeError("Folder already exists! Use '-ow' to overwrite")

    if not folder_exists:
        os.makedirs(output_folder)

    return output_folder, folder_name


def arg_parser() -> argparse.Namespace:
    """
    Parses program command line arguments

    Returns:
        argparse.Namespace: Parsed arguments
    """

    parser = argparse.ArgumentParser(description='Data unpacker')
    parser.add_argument('-m', '--mode', required=True, choices=['bag', 'live'],
                        help='Choose mode to process bag files or live topics')
    parser.add_argument('-b', '--bag', default=None,
                        help='The bag file path')
    parser.add_argument('-d', '--directory',
                        help='Directory to store the extracted frames. '
                        'Overrides the value in the configuration file.')
    parser.add_argument('-f', '--fps', type=int,
                        default=DEF_FPS, help='Save data using this fps')
    parser.add_argument('-cf', '--calib-file',
                        help="Path to camera calibration file to use for "
                             "sensor fusion. "
                             "The supported format is ROS' YAML format.")
    parser.add_argument('-2d', '--radar-2d', action='store_true',
                        help='Add this flag if radar is using 2D configuration')
    parser.add_argument('-ow', '--overwrite', action='store_true',
                        help='Overwrite the output directory')
    parser.add_argument('-p', '--progress',
                        action='store_true', help='Display progress bar')
    # parser.add_argument('-l', '--list', action='store_true',
    #                     help='Lists topics in the rosbag and exits')
    parser.add_argument('-c', '--config', default=DEF_CFG_PATH,
                        help='Path to configuration file')
    args = parser.parse_args()
    return args


def radar_lidar_cam_fusion_multiproc(args):
    radar_id, save_path, radar_seq, lidar_seq, cam_seq, proj_img, proj_pkl, \
        radar_cam_calib, tr_radar_lidar, euclidean_thr = args
    lidar_xyz, radar_xyz, radar_xyz_tr, lidar_radar_xyz, img = \
            loop_step(radar_id, save_path, radar_seq, lidar_seq, cam_seq,
                      tr_radar_lidar, euclidean_thr)

    # plot_lidar_radar(lidar_radar_xyz, radar_xyz, radar_xyz_tr, f_plot_out)
    # img_out = lidar_camera_projection.render_lidar_on_image(lidar_radar_xyz,
    #                                                         img,
    #                                                         lidar_cam_calib,
    #                                                         f_pkl_out)
    img_out = render_lidar_on_image(lidar_radar_xyz, img, radar_cam_calib,
                                    proj_pkl)

    cv2.imwrite(proj_img, img_out)


def lidar_cam_proj_multiproc(args):
    save_path, cam_seq, seq, proj_img_dir, proj_pkl_dir = args
    img = cv2.imread(
        os.path.join(save_path, 'camera', f'seq_{cam_seq}_rgb.png'),
        cv2.IMREAD_UNCHANGED)
    with open(
            os.path.join(save_path, 'lidar', f'seq_{seq}_lidar.pkl'),
            'rb') as f:
        lidar_points = pickle.load(f)
    img_out = os.path.join(proj_img_dir,
                           f'./lidar_{seq}_to_rgb_{cam_seq}.png')
    # Projected Lidar
    pkl_out = os.path.join(proj_pkl_dir,
                           f'./lidar_{seq}_to_rgb_{cam_seq}.pkl')
    # start = time.time()
    lidar_projection(img, lidar_points, pkl_out, img_out)
    # end = time.time()


def main(args: argparse.Namespace):
    """
    Main function

    Args:
        args argparse.Namespace: Arguments parsed by argparse
    """
    # Read config file
    cfg_path = os.path.abspath(os.path.expanduser(args.config))
    print(f"Using config file: {cfg_path}")
    try:
        cfg = read_cfg(cfg_path)
    except Exception as e:
        print(f"CFG Error! {str(e)}")
        exit(1)

    # Read the topics in database, specified in 'sensor'.
    db = DBHandler(cfg.get('db', dict()))
    db_topics = db.get_sensor_topics()

    output_path = args.directory if args.directory is not None \
        else cfg.get('data_path', '')
    output_path = os.path.expanduser(output_path)
    if output_path == '':
        print("No output folder specified!", file=sys.stderr)
        exit(1)

    if args.mode == 'bag':
        print('bag mode')
        if args.bag is None:
            raise RuntimeError('Please specify bag file for "bag unpack" mode!')
        else:
            bag_topics = BagParsing(args).read_topics()
            valid_topics = topic_check(bag_topics, db_topics)
            folder_name = Path(args.bag).stem
            save_path, folder = get_output_folder(folder_name, output_path,
                                                  args.overwrite)

            BagParsing(args, cfg, db, save_path,
                        folder, valid_topics).parse_rosbag()

            proj_img_dir = os.path.join(save_path, 'proj_img')
            proj_pkl_dir = os.path.join(save_path, 'proj_pkl')
            for d in [proj_img_dir, proj_pkl_dir]:
                if not os.path.exists(d):
                    os.mkdir(d)
            lc_csv_path = os.path.join(save_path, 'lidar-to-cam-seq-sync.csv')

            if os.path.exists(lc_csv_path):
                df_lidar_to_cam = pd.read_csv(lc_csv_path)

            else:
                RuntimeError("Lidar to camera seq sync file doesn't exist!")

            #bar_count = 0
            if args.progress:
                lidar_count = len(df_lidar_to_cam.lidar)
                bar = progressbar.ProgressBar(
                    max_value=lidar_count, prefix='Projecting lidar to camera: ',
                    redirect_stdout=True)
            lidar_seq = df_lidar_to_cam.lidar.values

            payload = []

            for seq in lidar_seq:
                cam_seq = df_lidar_to_cam.set_index('lidar').loc[
                    [seq]].values.reshape(-1)
                payload.append((save_path, cam_seq[-1], seq,
                                proj_img_dir, proj_pkl_dir))

            pool = Pool(8)
            t_start = time.time()
            pool.map(lidar_cam_proj_multiproc, payload)
            t_sec = time.time() - t_start
            pool.close()
            print(f'Parallel Lidar-Cam fusion takes {t_sec} sec')

            #     print('Average Time per frame of lidar projection:',
            #           (end - start) * 1000)
            #     if args.progress:
            #         bar_count += 1
            #         bar.update(bar_count)
            # if args.progress:
            #     bar.finish()

            proj_radar1_lidar_img_dir =\
                 os.path.join(save_path, 'proj_radar1_lidar_img')
            proj_radar1_lidar_pkl_dir =\
                 os.path.join(save_path, 'proj_radar1_lidar_pkl')
            proj_radar2_lidar_img_dir = \
                 os.path.join(save_path, 'proj_radar2_lidar_img')
            proj_radar2_lidar_pkl_dir = \
                 os.path.join(save_path, 'proj_radar2_lidar_pkl')

            for dir_path in [proj_radar1_lidar_img_dir,
                              proj_radar1_lidar_pkl_dir,
                              proj_radar2_lidar_img_dir,
                              proj_radar2_lidar_pkl_dir]:
                if not os.path.exists(dir_path):
                    print(f"Create dir: {dir_path}")
                    os.mkdir(dir_path)

            df_radar1_lidar_cam_sync =\
               pd.read_csv(os.path.join(save_path,
                                         'radar1-to-lidar-to-cam-seq-sync.csv'))

            df_radar2_lidar_cam_sync = \
                pd.read_csv(os.path.join(save_path,
                                         'radar2-to-lidar-to-cam-seq-sync.csv'))

            radar1_cam_calib =\
                read_calib_file(os.path.join('config', 'radar1_cam_calib_v2.txt'))

            radar2_cam_calib = \
                read_calib_file(
                    os.path.join('config', 'radar1_cam_calib_v2.txt'))

            tr_radar_lidar = np.asarray([[1., 0., 0., 0.3],
                                         [0., 1., 0., 0.2],
                                         [0., 0., 1., -0.2]])

            payload = []
            for i in df_radar1_lidar_cam_sync.index.values:
                radar_seq = i
                lidar_seq = df_radar1_lidar_cam_sync.set_index('radar1').\
                            loc[i].values.reshape(-1)[0]
                cam_seq = df_radar1_lidar_cam_sync.set_index('radar1'). \
                    loc[i].values.reshape(-1)[1]

                f_img_base = os.path.join(proj_radar1_lidar_img_dir,
                    f'radar1[{radar_seq}]_lidar[{lidar_seq}]_cam[{cam_seq}]')
                f_pkl_base = os.path.join(proj_radar1_lidar_pkl_dir,
                    f'radar1[{radar_seq}]_lidar[{lidar_seq}]_cam[{cam_seq}]')
                f_img_out = f'{f_img_base}.png'
                f_pkl_out = f'{f_pkl_base}.pkl'
                # f_plot_out = f'{f_base}.png'
                euclidean_thr = cfg.get("radar_lidar_euclidean_thr")

                payload.append(('radar1', save_path, radar_seq, lidar_seq,
                                cam_seq, f_img_out, f_pkl_out,radar1_cam_calib,
                                tr_radar_lidar, euclidean_thr))

            pool = Pool(8)
            t_start_radar1 = time.time()
            pool.map(radar_lidar_cam_fusion_multiproc, payload)
            t_sec_radar1 = time.time() - t_start_radar1
            pool.close()

            payload = []
            for i in df_radar2_lidar_cam_sync.index.values:
                radar_seq = i
                lidar_seq = df_radar2_lidar_cam_sync.set_index('radar2'). \
                    loc[i].values.reshape(-1)[0]
                cam_seq = df_radar2_lidar_cam_sync.set_index('radar2'). \
                    loc[i].values.reshape(-1)[1]

                f_img_base = os.path.join(proj_radar2_lidar_img_dir,
                    f'radar2[{radar_seq}]_lidar[{lidar_seq}]_cam[{cam_seq}]')
                f_pkl_base = os.path.join(proj_radar2_lidar_pkl_dir,
                    f'radar2[{radar_seq}]_lidar[{lidar_seq}]_cam[{cam_seq}]')
                f_img_out = f'{f_img_base}.png'
                f_pkl_out = f'{f_pkl_base}.pkl'
                # f_plot_out = f'{f_base}.png'
                euclidean_thr = cfg.get("radar_lidar_euclidean_thr")
                payload.append(('radar2', save_path, radar_seq, lidar_seq,
                                cam_seq, f_img_out, f_pkl_out, radar2_cam_calib,
                                tr_radar_lidar, euclidean_thr))

            pool = Pool(8)
            t_start_radar2 = time.time()
            pool.map(radar_lidar_cam_fusion_multiproc, payload)
            t_sec_radar2 = time.time() - t_start_radar2
            pool.close()
            print(f'Parallel radar-lidar-cam fusion for two radar sensors '
                  f'takes {t_sec_radar1 + t_sec_radar2} sec')


    # elif args.mode == 'live':
    #   # TODO: need to check whether topics in cfg really available in pipeline
    #     print('live mode')
    #     live_topics = cfg.get('live_topics')
    #     valid_topics = topic_check(live_topics, db_topics)
    #     print(valid_topics)
    #     folder_name = time.strftime("%Y-%d-%m-%H-%M-%S", time.localtime())
    #     save_path, folder = get_output_folder(folder_name, output_path,
    #                                           args.overwrite)
    #     #LiveParsing()
    else:
        exit()





    # raspi_cfg = cfg.get('raspi', dict())
    # camera = Camera(raspi_cfg['raspi_ground_height'],
    #                 raspi_cfg['raspi_angle'],
    #                 raspi_cfg['raspi_calib_intri'],
    #                 raspi_cfg['raspi_calibration_size'])
    # print(camera)
    # print(valid_topics)
    #DataProcess(args, cfg, db, save_path, folder, valid_topics).save_to_db()

    # Process rosbag
    # bag_parser = RosbagParser(args, db, save_path, folder,
    #                           valid_topics=valid_topics)
    # bag_parser.parse_rosbag(camera)

    db.close()


if __name__ == '__main__':
    # Parse arguments
    args = arg_parser()

    # Start the main program
    main(args)
