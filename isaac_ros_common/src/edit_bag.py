#!/bin/python

import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message
import math

unwanted_topics = ['mavros/cam_imu_sync/cam_imu_stamp'
'/mavros/camera/image_captured'
'/mavros/estimator_status'
'/mavros/extended_state'
'/mavros/geofence/fences',
'/mavros/gimbal_control/device/attitude_status',
'/mavros/gimbal_control/device/info',
'/mavros/gimbal_control/manager/info',
'/mavros/gimbal_control/manager/status',
'/mavros/landing_target/lt_marker',
'/mavros/landing_target/pose_in',
'/mavros/log_transfer/raw/log_data',
'/mavros/log_transfer/raw/log_entry',
'/mavros/mag_calibration/report',
'/mavros/mag_calibration/status',
'/mavros/manual_control/control',
'/mavros/mission/reached',
'/mavros/mission/waypoints',
'/mavros/mount_control/orientation',
'/mavros/mount_control/status',
'/mavros/nav_controller_output/output',
'/mavros/param/event',
'/mavros/radio_status',
'/mavros/rallypoint/rallypoints',
'/mavros/rangefinder/rangefinder',
'/mavros/rangefinder_pub',
'/mavros/rc/in',
'/mavros/rc/out',
'/mavros/setpoint_raw/target_attitude',
'/mavros/setpoint_raw/target_global',
'/mavros/setpoint_raw/target_local',
'/mavros/setpoint_trajectory/desired',
'/mavros/state',
'/mavros/status_event',
'/mavros/statustext/recv',
'/mavros/sys_status',
'/mavros/terrain/report',
'/mavros/time_reference',
'/mavros/timesync_status',
'/mavros/trajectory/desired',
'/mavros/tunnel/out',
'/mavros/vfr_hud',
'/move_base_simple/goal']
topics_to_adjust = ['/ouster/points', '/ouster/imu', '/left/image_raw']

first=True
altosBagTimestamp = float(0)
altosMsgTimestamp = float(0)

reader = rosbag2_py.SequentialReader()
reader.open(
    rosbag2_py.StorageOptions(uri='rosbag', storage_id='sqlite3'),
    rosbag2_py.ConverterOptions(input_serialization_format='cdr',
                                output_serialization_format='cdr')
)

writer = rosbag2_py.SequentialWriter()
writer.open(
    rosbag2_py.StorageOptions(uri='final_bag', storage_id='sqlite3'),
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
    topic, data, timestamp = reader.read_next()
    filter_out = False
    if topic in unwanted_topics:
        filter_out = True
    elif topic == '/altos/altosRadar':
        altosBagTimestamp = float(timestamp)/1e9
        msg_type = get_message(typename(topic))
        msg = deserialize_message(data, msg_type)
        altosMsgTimestamp = float(msg.header.stamp.sec)+float(msg.header.stamp.nanosec)/float(1e9)
        first=False
    elif topic in topics_to_adjust:
        bag_time_offset = altosBagTimestamp - float(timestamp)/1e9
        msg_type = get_message(typename(topic))
        msg = deserialize_message(data, msg_type)
        if first:
            raise Exception("DIDNT GET ALTOS MESSAGE FIRST") 
        ousterMsgTimestamp = altosMsgTimestamp - bag_time_offset
        msg.header.stamp.sec = int(math.floor(ousterMsgTimestamp))
        msg.header.stamp.nanosec = int((ousterMsgTimestamp % 1)*1e9)
        data = serialize_message(msg)
    elif topic=='/tf':
        msg_type = get_message(typename(topic))
        msg = deserialize_message(data, msg_type)
        for transform in msg.transforms:
            if transform.header.frame_id == 'camera':
                filter_out = True
                break
        
    if not filter_out:
        writer.create_topic(rosbag2_py.TopicMetadata(name=topic, type=typename(topic), serialization_format="cdr"))
        writer.write(topic, data, timestamp)

