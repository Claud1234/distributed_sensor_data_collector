from contextlib import redirect_stdout
import rosbag
import rospy
import os

from collections import namedtuple
from dataclasses import dataclass
import progressbar
from pathlib import Path
import shutil

from src.topics import Topics, TopicData, LastFrame
from src.frame import process_frame
       

class RosbagParser():
    """
    Class for parsing rosbags
    """

    def __init__(self, args: str, topics: Topics=None) -> None:
        """
        Constructor

        Args:
            bag (str): Program's arguments
            topics (Topics, optional): Object containing all topics to read. 
                                       If None, the topics are guessed automatically 
                                       at object's creation time. Defaults to None.
        """

        self.args = args

        self.bag = rosbag.Bag(args.rosbag)

        if topics is None:
            self.topics = self._guess_topics()
        else:
            self.topics = topics

        self.output_folder = self._get_output_folder()

        self.last_frame_msg = LastFrame(0, 0, None, None, None, None)
        self.freq = int((1/args.fps) * 1e9)

        print(f"Saving frames after every {self.freq} ns ({self.freq / 1e9} s, {args.fps} FPS)")

    def __del__(self) -> None:
        """
        Destructor
        """
        self.bag.close()

    def _topic_to_obj(self, name: str, params: namedtuple) -> TopicData:
        """
        Converts rosbag formatted topic info to TopicData object

        Args:
            name (str): Topic name
            params (namedtuple): Rosbag's TopicTuple object

        Returns:
            TopicData: Topic information as TopicData object
        """
        obj = TopicData(name,
                        params.msg_type,
                        params.message_count,
                        params.connections,
                        params.frequency)

        return obj

    def _guess_topics(self) -> Topics:
        """
        Tries to guess in which groups the topics in the rosbag belongs to

        Returns:
            Topics: Topics object containing all topics filtered in to groups
        """

        bag_topics = self.bag.get_type_and_topic_info().topics

        image_topics = []
        radar_pc = []
        lidar_pc = []
        gps_fix = []
        transforms = []

        for name, params in bag_topics.items():
            obj = self._topic_to_obj(name, params)

            if obj.type == 'sensor_msgs/CompressedImage':
                image_topics.append(obj)

            elif obj.type == 'sensor_msgs/NavSatFix':
                gps_fix.append(obj)

            elif obj.type == 'tf2_msgs/TFMessage':
                transforms.append(obj)

            elif obj.type == 'sensor_msgs/PointCloud2':
                if 'radar' in obj.name:
                    radar_pc.append(obj)

                elif 'lidar' in obj.name:
                    lidar_pc.append(obj)

        return Topics(image_topics, 
                      radar_pc, 
                      lidar_pc, 
                      gps_fix, 
                      transforms)

    def _get_output_folder(self) -> str:
        output_folder = ''

        bag_name = Path(self.args.rosbag).stem
        output_folder = os.path.join(self.args.output, bag_name)
        folder_exists = os.path.exists(output_folder)

        if folder_exists:
            if self.args.overwrite:
                shutil.rmtree(output_folder)
                folder_exists = False
            else:
                print('Error! Folder already exists!')
                exit(1)

        if not folder_exists:
            os.makedirs(output_folder)
            print("Output directory created!")

        return output_folder

    def parse_rosbag(self):
        if self.args.progress:
            msg_count = self.bag.get_message_count()
            bar = progressbar.ProgressBar(max_value=msg_count, prefix='Processing: ', redirect_stdout = True)
            
        i = 0
        for topic, msg, t in self.bag.read_messages():
            if self.topics.is_topic_in_group(topic, 'images'):
                self.last_frame_msg.image = msg.data
            
            elif self.topics.is_topic_in_group(topic, 'radar_pc'):
                self.last_frame_msg.radar_pc = msg

            elif self.topics.is_topic_in_group(topic, 'lidar_pc'):
                self.last_frame_msg.lidar_pc = msg

            elif self.topics.is_topic_in_group(topic, 'gps_fix'):
                self.last_frame_msg.gps_fix = msg

            # elif self.topics.is_topic_in_group(topic, 'transforms'):
            #     print(msg)#self.last_frame_msg. = msg.data

            time_ns = rospy.rostime.Time.to_nsec(t)

            if self.args.progress:
                bar.update(i)

            if time_ns >= self.last_frame_msg.timestamp + self.freq:

                if self.last_frame_msg.timestamp > 0:
                    self.last_frame_msg.frame_counter += 1

                    if self.last_frame_msg.has_enough_data(self.topics):
                        process_frame(self.last_frame_msg, self.output_folder)

                self.last_frame_msg.timestamp = time_ns

            i += 1

        if self.args.progress:
            bar.finish()
