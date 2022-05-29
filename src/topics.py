from itertools import chain
from dataclasses import dataclass


@dataclass
class TopicData:
    """
    Class to store data for each topic
    """
    name: str
    type: str
    message_count: int
    connections: int
    frequency: float


@dataclass
class Topics:
    """
    Class to store and sort all topics
    """
    images: list
    radar_pc: list
    lidar_pc: list
    gps_fix: list
    transforms: list

    def as_dict(self) -> dict():
        return {
                'images': self.images,
                'radar_pc': self.radar_pc,
                'lidar_pc': self.lidar_pc,
                'gps_fix': self.gps_fix,
                'transforms': self.transforms
                }

    def get_all_topics(self) -> list:
        """
        Returns the list of all topic objects

        Returns:
            list: List of TopicData elements
        """
        return list(chain(self.images, 
                          self.radar_pc,
                          self.lidar_pc,
                          self.gps_fix,
                          self.transforms))

    def get_topic_names(self, topic_list: list=None) -> list:
        """
        List of the names of topics

        Args:
            topic_list (list, optional): List of topics to process.
                                         If None, all topics in the class will be processed.
                                         Defaults to None.

        Returns:
            list: List of strings containing topic names
        """

        if topic_list is None:
            topic_list = self.get_all_topics()

        return [topic.name for topic in topic_list]

    def is_topic_in_group(self, topic_name: str, topic_group: str) -> bool:
        """
        Checks if topic is in a specific topic group

        Args:
            topic_name (str): Name of the topic to check
            topic_group (str): Group to check

        Raises:
            ValueError: Raised when an invalid topic group is passed

        Returns:
            bool: True of topic is in the group, False otherwise
        """

        topic_groups = {
            'images': self.images,
            'radar_pc': self.radar_pc,
            'lidar_pc': self.lidar_pc,
            'gps_fix': self.gps_fix,
            'transforms': self.transforms
        }

        # Invalid group
        if not topic_group in topic_groups.keys():
            raise ValueError(f"Invalid topic group: {topic_group}")

        return topic_name in self.get_topic_names(topic_list=topic_groups[topic_group])

    def get_topic_data_by_name(self, topic_name: str) -> TopicData:
        """
        Gets a topic based on topic name

        Args:
            topic_name (str): Topic name to search for

        Returns:
            TopicData: Data for the topic, if found. None otherwise
        """
        all_topics = self.get_all_topics()
        valid_names = self.get_topic_names()

        if topic_name not in valid_names: return None

        topic_list = list(filter(lambda topic: topic.name == topic_name, all_topics))

        # We this should never return more than 1 results. If it does, something is wrong
        if len(topic_list) != 1: raise(ValueError("Error! Two topics with same name?"))

        return topic_list[0]

@dataclass
class LastFrame:
    """
    Object for storing storing received data.
    """
    frame_counter: 0
    timestamp: None
    images: dict()
    radar_pc: dict()
    lidar_pc: dict()
    gps_fix: None

    def has_enough_data(self, topics: Topics) -> bool:
        """
        Checks if we have collected data from all topics that
        we are monitoring.

        Args:
            topics (Topics): Topics object containing valid topics

        Returns:
            bool: True if enough data has been collected. False otherwise
        """

        if len(topics.images) > 0 and len(self.images) != len(topics.images):
            return False

        if len(topics.radar_pc) > 0 and len(self.radar_pc) != len(topics.radar_pc):
            return False

        if len(topics.lidar_pc) > 0 and len(self.lidar_pc) != len(topics.lidar_pc):
            return False

        if len(topics.gps_fix) > 0 and self.gps_fix is None:
            return False


        return True