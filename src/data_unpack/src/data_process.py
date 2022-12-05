#!/usr/bin/env python3
"""
Process data from bag file or live topics, then save in database
"""
import os
import rospy
import rosbag
import progressbar
import itertools

from src.camera import Camera
from src.topics import LastFrame
from src.frame import process_frame


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
            self.raspi_cam_r = SensorParam(cfg).raspi_cam()

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
        bag_file = rosbag.Bag(self.args.bag)
        # # Timestamp for the file
        # timestamp = os.path.getmtime(self.args.bag)

        i = 0
        # Set up progressbar
        if self.args.progress:
            msg_count = bag_file.get_message_count()
            bar = progressbar.ProgressBar(max_value=msg_count,
                                            prefix='Processing: ',
                                            redirect_stdout=True)

        # Read rosbag topics
        for topic, msg, t in bag_file.read_messages():
            # Sensor is a camera
            if topic in self.valid_topics.get('camera', []):
                self.last_frame_msg.data[topic] = msg.data

            # Sensor is a radar
            elif topic in self.valid_topics.get('radar', []):
                self.last_frame_msg.data[topic] = msg

            elif topic in self.valid_topics.get('lidar', []):
                self.last_frame_msg.data[topic] = msg

            time_ns = rospy.rostime.Time.to_nsec(t)

            if self.args.progress:
                bar.update(i)

            # For syncing to framerate
            if time_ns >= self.last_frame_msg.timestamp + self.freq:

                if self.last_frame_msg.timestamp > 0:
                    self.last_frame_msg.frame_counter += 1

                    # If we have collected enough data for the frame
                    valid_topic_list = list(itertools.chain.from_iterable(
                                            list(self.valid_topics.values())))
                    if self.last_frame_msg.has_enough_data(valid_topic_list):
                        # Save the frame data
                        frame_sensors = process_frame(self.raspi_cam_r,
                                                      self.last_frame_msg,
                                                      self.save_path,
                                                      self.folder_name,
                                                      self.args.radar_2d,
                                                      self.valid_topics)
                        if self.db:
                            self.db.save_frame(time_ns / 1e9, frame_sensors)

                self.last_frame_msg.timestamp = time_ns

            i += 1

        if self.args.progress:
            bar.finish()


class LiveParsing(object):
    def __init__(self, args,
                 cfg='', db='', save_path='', folder='', valid_topics=''):
        self.args = args
        self.cfg = cfg
        self.db = db
        self.save_path = save_path
        self.folder = folder
        self.topics = valid_topics

        self.bag_topics = []

        # self.last_frame_msg = LastFrame(0, 0, dict())
        self.freq = int((1 / args.fps) * 1e9)

        # Get camera's rotational matrix for radar projection
        raspi_cfg = self.cfg.get('raspi', dict())
        self.camera = Camera(raspi_cfg['raspi_ground_height'],
                             raspi_cfg['raspi_angle'],
                             raspi_cfg['raspi_calib_intri'],
                             raspi_cfg['raspi_calibration_size'])

    def parse_live_topics(self):
        return


