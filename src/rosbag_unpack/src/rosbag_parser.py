import argparse
import itertools
import os

import progressbar
import rosbag
import rospy

from src.camera import Camera
from src.db import DBHandler
from src.frame import process_frame
from src.topics import LastFrame


class RosbagParser():
    """
    Class for parsing rosbags
    """

    def __init__(self, args: argparse.Namespace, db: DBHandler = None, 
                 save_path: str = '', folder: str = '', 
                 valid_topics: dict = None) -> None:
        """
        Constructor

        Args:
            bag (argparse.Namespace): Program's arguments
            valid_topics (dict, optional): Object containing all topics to read. 
                                       If None, the topics are guessed automatically 
                                       at object's creation time. Defaults to None.
        """

        self.args = args

        self.bag = rosbag.Bag(args.rosbag)
        self.topics = valid_topics
        self.save_path = save_path
        self.folder = folder

        self.db = db

        # Timestamp for the file
        self.timestamp = os.path.getmtime(args.rosbag)

        self.last_frame_msg = LastFrame(0, 0, dict())
        self.freq = int((1/args.fps) * 1e9)

        self.bag_topics = []

        print(f"Saving frames after every {self.freq} ns ({self.freq / 1e9} s, {args.fps} FPS)")

    def __del__(self) -> None:
        """
        Destructor
        """
        self.bag.close()

    def get_valid_topics(self) -> dict:
        """
        Returns dictionary containing valid topics sorted by
        sensor type

        Returns:
            dict: Dictionary containing topics sorted by
                  sensor type
        """
        return self.topics

    def get_valid_topic_list(self) -> list:
        """
        Returns a list of all valid topics

        Returns:
            dict: List of all valid topics
        """
        return list(itertools.chain.from_iterable(list(self.topics.values())))
    
    def get_topics(self) -> dict:
        """
        Retuns topics in rosbag

        Returns:
            dict: Dictionary containing the rosbag topics
        """
        return self.bag_topics

    def print_topics(self) -> None:
        """
        Prints rosbag topics on screen
        """
        
        # Print results
        print("\nTOPICS:")
        print("=" * 10)

        for topic in self.bag_topics:
            print(topic)

        print()

    def read_topics(self) -> None:
        """
        Reads all topics from a rosbag file
        """

        i = 0

        # Progressbar
        if self.args.progress:
            msg_count = self.bag.get_message_count()
            bar = progressbar.ProgressBar(
                max_value=msg_count, prefix='Analyzing rosbag: ', 
                        redirect_stdout=True)

        # Read topics
        for topic, _, _ in self.bag.read_messages():
            if topic not in self.bag_topics:
                self.bag_topics.append(topic)

            # Update progressbar
            if self.args.progress:
                i += 1
                bar.update(i)

        if self.args.progress:
            bar.finish()

    def parse_rosbag(self, camera: Camera) -> None:
        """
        Parses a rosbag file
        """
        
        i = 0

        # Set up progressbar
        if self.args.progress:
            msg_count = self.bag.get_message_count()
            bar = progressbar.ProgressBar(
                max_value=msg_count, prefix='Processing: ', redirect_stdout=True)

        # Read rosbag topics
        for topic, msg, t in self.bag.read_messages():
            
            # Sensor is a camera
            if topic in self.topics.get('camera', []):
                self.last_frame_msg.data[topic] = msg.data

            # Sensor is a radar
            elif topic in self.topics.get('radar', []):
                self.last_frame_msg.data[topic] = msg

            time_ns = rospy.rostime.Time.to_nsec(t)

            if self.args.progress:
                bar.update(i)

            # For syncing to framerate
            if time_ns >= self.last_frame_msg.timestamp + self.freq:

                if self.last_frame_msg.timestamp > 0:
                    self.last_frame_msg.frame_counter += 1

                    # If we have collected enough data for the frame
                    if self.last_frame_msg.has_enough_data(self.get_valid_topic_list()):
                        # Save the frame data
                        frame_sensors = process_frame(camera, self.last_frame_msg, self.save_path, 
                                            self.folder, self.args.radar_2d, self.get_valid_topics())
                        if self.db:
                            self.db.save_frame(time_ns / 1e9, frame_sensors)

                self.last_frame_msg.timestamp = time_ns

            i += 1

        if self.args.progress:
            bar.finish()
