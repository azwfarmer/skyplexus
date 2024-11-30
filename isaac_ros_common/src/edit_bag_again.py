#!/bin/python

import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message
import math

reader = rosbag2_py.SequentialReader()
reader.open(
    rosbag2_py.StorageOptions(uri='rosbag2_2024_10_05-18_01_21', storage_id='sqlite3'),
    rosbag2_py.ConverterOptions(input_serialization_format='cdr',
                                output_serialization_format='cdr')
)

writer = rosbag2_py.SequentialWriter()
writer.open(
    rosbag2_py.StorageOptions(uri='edited_rosbag2_2024_10_05-18_01_21', storage_id='sqlite3'),
    rosbag2_py.ConverterOptions(input_serialization_format='cdr',
                                output_serialization_format='cdr')
)

topic_types = reader.get_all_topics_and_types()

def typename(topic_name):
    for topic_type in topic_types:
        if topic_type.name == topic_name:
            return topic_type.type
    raise ValueError(f"topic {topic_name} not in bag")

while reader.has_next():
    filter_out = False
    topic, data, timestamp = reader.read_next()
    if topic=='/tf':
        msg_type = get_message(typename(topic))
        msg = deserialize_message(data, msg_type)
        for transform in msg.transforms:
            if transform.header.frame_id == 'map' and transform.child_frame_id == 'base_link':
                if(abs(transform.transform.translation.x) > 1e4 or abs(transform.transform.translation.y) > 1e4 or abs(transform.transform.translation.z) > 1e4):
                    print("problem discovered")
                    filter_out = True
                    break
                
    if not filter_out:
        writer.create_topic(rosbag2_py.TopicMetadata(name=topic, type=typename(topic), serialization_format="cdr"))
        writer.write(topic, data, timestamp)

