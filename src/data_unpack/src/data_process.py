#!/usr/bin/env python3
"""
Process data from bag file or live topics, then save in database
"""
import os
import cv2
import pickle
import json
import struct
import numpy as np
import pandas as pd
import rospy
import rosbag
import progressbar
import itertools
import velodyne_decoder as vd
from velodyne_decoder_pylib import *
from message_filters import Subscriber, TimeSynchronizer, ApproximateTimeSynchronizer


from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from src.camera import Camera
from src.topics import LastFrame


def get_radar_lidar_cam_sync(df_radar_lidar_sync, df_lidar_cam_sync,
                             radar_col='radar1',
                             lidar_col='lidar'):
    """Index is radar"""
    return df_radar_lidar_sync.set_index(radar_col)\
        .join(df_lidar_cam_sync.set_index(lidar_col), on=lidar_col)


class SensorParam(object):
    def __init__(self, cfg):
        self.cfg = cfg

    def raspi_cam(self):
        # Get camera's rotational matrix for radar projection
        raspi_cfg = self.cfg.get('raspi', dict())
        raspi_r = Camera(raspi_cfg['raspi_ground_height'],
                             raspi_cfg['raspi_angle'],
                             raspi_cfg['raspi_calib_intri'],
                             raspi_cfg['raspi_calibration_size'])
        return raspi_r


class BagParsing(object):
    """
       Class to parses a rosbag file to database
    """
    def __init__(self, args,
                 cfg=None, db=None, save_path='', folder='',
                 valid_topics: dict = None):
        self.args = args
        self.db = db
        self.save_path = save_path
        self.folder_name = folder
        self.valid_topics = valid_topics

        self.bag_file = rosbag.Bag(self.args.bag)
        self.bag_topics = []
        if cfg is not None:
            self.cfg = cfg
            #self.raspi_cam_r_matrix = SensorParam(cfg).raspi_cam()

        self.last_frame_msg = LastFrame(0, 0, dict())
        self.freq = int((1 / args.fps) * 1e9)

    def read_topics(self):
        """
        Reads all topics from a rosbag file
        """

        i = 0
        # Progressbar
        if self.args.progress:
            msg_count = self.bag_file.get_message_count()
            bar = progressbar.ProgressBar(
                max_value=msg_count, prefix='Loading bag file: ',
                redirect_stdout=True)

        # Read topics
        for topic, _, _ in self.bag_file.read_messages():
            if topic not in self.bag_topics:
                self.bag_topics.append(topic)

            # Update progressbar
            if self.args.progress:
                i += 1
                bar.update(i)

        if self.args.progress:
            bar.finish()
        print("\nTopics in bag:")
        print(self.bag_topics)
        return self.bag_topics
        
    def parse_rosbag(self):
        config = vd.Config(model='VLP-32C')
        decoder = ScanDecoder(config)
        bag = rosbag.Bag(self.args.bag)
        # # Timestamp for the file
        # timestamp = os.path.getmtime(self.args.bag)

        lidar_msg_gen = bag.read_messages(topics=self.cfg.get('lidar_topic'))
        cam_msg_gen = bag.read_messages(topics=self.cfg.get('image_topic'))
        radar1_msg_gen = bag.read_messages(topics=self.cfg.get('radar_1_topic'))
        radar2_msg_gen = bag.read_messages(topics=self.cfg.get('radar_2_topic'))

        np_nsec_lidar = np.asarray(
            [msg.timestamp.to_nsec() for msg in lidar_msg_gen])
        np_nsec_cam = np.asarray(
            [msg.timestamp.to_nsec() for msg in cam_msg_gen])
        np_nsec_radar1 = np.asarray(
            [msg.timestamp.to_nsec() for msg in radar1_msg_gen])
        np_nsec_radar2 = np.asarray(
            [msg.timestamp.to_nsec() for msg in radar2_msg_gen])

        np_ts_match_lc = np.zeros((len(np_nsec_cam), len(np_nsec_lidar)),
                                  np.int64)
        np_ts_match_r1l = np.zeros((len(np_nsec_lidar), len(np_nsec_radar1)),
                                   np.int64)
        np_ts_match_r2l = np.zeros((len(np_nsec_lidar), len(np_nsec_radar2)),
                                   np.int64)

        for i, tc in enumerate(np_nsec_cam):
            for j, tl in enumerate(np_nsec_lidar):
                np_ts_match_lc[i, j] = tc - tl
        for i, tl in enumerate(np_nsec_lidar):
            for j, tr0 in enumerate(np_nsec_radar1):
                np_ts_match_r1l[i, j] = tl - tr0
        for i, tl in enumerate(np_nsec_lidar):
            for j, tr0 in enumerate(np_nsec_radar2):
                np_ts_match_r2l[i, j] = tl - tr0
        np_ts_match_lc = np.abs(np_ts_match_lc)
        np_ts_match_r1l = np.abs(np_ts_match_r1l)
        np_ts_match_r2l = np.abs(np_ts_match_r2l)

        df_seq_match_lc = \
            pd.DataFrame(dict(list(zip(['lidar', 'cam'], np.vstack(
                [np.arange(np_nsec_lidar.shape[0]),
                 np.argmin(np_ts_match_lc.T, axis=1)])))))
        df_seq_match_lc.to_csv(self.save_path+'/lidar-to-cam-seq-sync.csv',
                                index=False)

        df_seq_match_r1l = \
            pd.DataFrame(dict(list(zip(['radar1', 'lidar'], np.vstack(
            [np.arange(np_nsec_radar1.shape[0]),
             np.argmin(np_ts_match_r1l.T, axis=1)])))))
        df_seq_match_r1l.to_csv(self.save_path+'/radar1-to-lidar-seq-sync.csv',
                                index=False)

        df_seq_match_r2l = \
            pd.DataFrame(dict(list(zip(['radar1', 'lidar'], np.vstack(
                [np.arange(np_nsec_radar2.shape[0]),
                 np.argmin(np_ts_match_r2l.T, axis=1)])))))
        df_seq_match_r2l.to_csv(
            self.save_path + '/radar2-to-lidar-seq-sync.csv',
            index=False)

        df_seq_match_r1lc =\
            get_radar_lidar_cam_sync(df_lidar_cam_sync=df_seq_match_lc,
                                     df_radar_lidar_sync=df_seq_match_r1l,
                                     radar_col='radar1')
        df_seq_match_r1lc\
            .to_csv(self.save_path + '/radar1-to-lidar-to-cam-seq-sync.csv')

        df_seq_match_r2lc = \
            get_radar_lidar_cam_sync(df_lidar_cam_sync=df_seq_match_lc,
                                     df_radar_lidar_sync=df_seq_match_r2l,
                                     radar_col='radar2')
        df_seq_match_r2lc \
            .to_csv(self.save_path + '/radar2-to-lidar-to-cam-seq-sync.csv')

        lidar_dir = os.path.join(self.save_path, 'lidar')
        cam_dir = os.path.join(self.save_path, 'camera')
        radar1_dir = os.path.join(self.save_path, 'radar1')
        radar2_dir = os.path.join(self.save_path, 'radar2')

        for d in [lidar_dir, cam_dir, radar1_dir, radar2_dir]:
            if not os.path.exists(d):
                print(f'mkdir {d}')
                os.mkdir(d)

        prog_bar_counter = 0
        # Set up progressbar
        if self.args.progress:
            msg_count = bag.get_message_count()
            bar = progressbar.ProgressBar(max_value=msg_count,
                                            prefix='Saving Raw Dara: ',
                                            redirect_stdout=True)

        # Read rosbag topics
        cam_seq = 0
        lidar_seq = 0
        radar1_seq = 0
        radar2_seq = 0
        for topic, msg, t in bag.read_messages():

            if topic == self.cfg.get('image_topic'):
                #seq = int(msg.header.seq)
                fn_seq = f'seq_{cam_seq}'
                rgb = f'{fn_seq}_rgb.png'
                rgb_path = os.path.join(cam_dir, rgb)
                rgb_array = cv2.imdecode(np.frombuffer(msg.data, np.uint8), -1)
                cv2.imwrite(rgb_path, rgb_array)
                rgb_db_info = {
                    'topic': self.cfg.get('image_topic'),
                    'file': rgb_path,
                    'data': None
                }
                cam_seq += 1

            elif topic == self.cfg.get('lidar_topic'):
                #seq = int(msg.header.seq)
                fn_seq = f'seq_{lidar_seq}'
                points = decoder.decode_message(msg, as_pcl_structs=False)
                lidar = f'{fn_seq}_lidar.pkl'
                lidar_path = os.path.join(lidar_dir, lidar)
                with open(lidar_path, 'wb') as f:
                    pickle.dump(points, f)
                lidar_db_info = {
                    'topic': self.cfg.get('lidar_topic'),
                    'file': lidar_path,
                    'data': None
                }
                lidar_seq += 1

            elif topic == self.cfg.get('radar_1_topic'):
                points = []
                fn_seq = f'seq_{radar1_seq}'
                radar1 = f'{fn_seq}_radar1.json'
                radar1_path = os.path.join(radar1_dir, radar1)
                i = 0
                for _ in range(msg.width * msg.height):
                    point_data = msg.data[i:(i + msg.point_step - 1)]
                    i += msg.point_step
                    point = dict()
                    for field in msg.fields:
                        format_str = '<f'
                        ba = bytearray(
                            point_data[field.offset:field.offset + 4])
                        data = struct.unpack(format_str, ba)
                        point[field.name] = data
                    points.append(point)
                    with open(radar1_path, 'w') as f:
                        json.dump(points, f, indent=4, sort_keys=True)
                    radar_1_db_info = {
                        'topic': self.cfg.get('radar_1_topic'),
                        'file': radar1_path,
                        'data': None
                    }
                radar1_seq += 1

            elif topic == self.cfg.get('radar_2_topic'):
                points = []
                fn_seq = f'seq_{radar2_seq}'
                radar2 = f'{fn_seq}_radar2.json'
                radar2_path = os.path.join(radar2_dir, radar2)
                i = 0
                for _ in range(msg.width * msg.height):
                    point_data = msg.data[i:(i + msg.point_step - 1)]
                    i += msg.point_step
                    point = dict()
                    for field in msg.fields:
                        format_str = '<f'
                        ba = bytearray(
                            point_data[field.offset:field.offset + 4])
                        data = struct.unpack(format_str, ba)
                        point[field.name] = data
                    points.append(point)
                    with open(radar2_path, 'w') as f:
                        json.dump(points, f, indent=4, sort_keys=True)
                    radar_2_db_info = {
                        'topic': self.cfg.get('radar_2_topic'),
                        'file': radar2_path,
                        'data': None
                    }
                radar2_seq += 1

            if self.args.progress:
                bar.update(prog_bar_counter)
            prog_bar_counter += 1

        if self.args.progress:
            bar.finish()



# class LiveParsing(object):
#     def __init__(self, args,
#                  cfg='', db='', save_path='', folder='', valid_topics=''):
#
#         self.valid_topics = valid_topics
#         rospy.init_node('live_topic_unpack', anonymous=True)
#         raspi_cam_sub = Subscriber(self.valid_topics.get('camera')[0], Image)
#         radar_1_sub = Subscriber(self.valid_topics.get('radar')[0], PointCloud2)
#         lidar_sub = Subscriber(self.valid_topics.get('lidar')[0], PointCloud2)
#
#         sync = ApproximateTimeSynchronizer([raspi_cam_sub,
#                                             radar_1_sub,
#                                             lidar_sub], queue_size=10, slop=0.1)
#         sync.registerCallback(self.parse_live_topics)
#
#         rospy.spin()
#         self.args = args
#         self.cfg = cfg
#         self.db = db
#         self.save_path = save_path
#         self.folder = folder
#         self.topics = valid_topics
#
#         self.bag_topics = []
#
#         # self.last_frame_msg = LastFrame(0, 0, dict())
#         self.freq = int((1 / args.fps) * 1e9)
#
#         # Get camera's rotational matrix for radar projection
#         raspi_cfg = self.cfg.get('raspi', dict())
#         self.camera = Camera(raspi_cfg['raspi_ground_height'],
#                              raspi_cfg['raspi_angle'],
#                              raspi_cfg['raspi_calib_intri'],
#                              raspi_cfg['raspi_calibration_size'])
#
#     def parse_live_topics(self, raspi_cam, radar1, lidar):
#         return

