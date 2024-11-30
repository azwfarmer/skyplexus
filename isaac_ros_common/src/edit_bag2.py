#!/bin/python

import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message
import math

unwanted_topics = [
'/mavros/adsb/vehicle',
'/mavros/battery',
'/mavros/cam_imu_sync/cam_imu_stamp',
'/mavros/camera/image_captured',
'/mavros/esc_status/info',
'/mavros/esc_status/status',
'/mavros/esc_telemetry/telemetry',
'/mavros/estimator_status',
'/mavros/extended_state',
'/mavros/geofence/fences',
'/mavros/gimbal_control/device/attitude_status',
'/mavros/gimbal_control/device/info',
'/mavros/gimbal_control/manager/info',
'/mavros/gimbal_control/manager/status',
'/mavros/global_position/compass_hdg',
'/mavros/global_position/global',
'/mavros/global_position/gp_lp_offset',
'/mavros/global_position/gp_origin',
'/mavros/global_position/local',
'/mavros/global_position/raw/fix',
'/mavros/global_position/raw/gps_vel',
'/mavros/global_position/raw/satellites',
'/mavros/global_position/rel_alt',
'/mavros/gps_rtk/rtk_baseline',
'/mavros/gpsstatus/gps1/raw',
'/mavros/gpsstatus/gps1/rtk',
'/mavros/gpsstatus/gps2/raw',
'/mavros/gpsstatus/gps2/rtk',
'/mavros/home_position/home',
'/mavros/imu/data',
'/mavros/imu/data_raw',
'/mavros/imu/diff_pressure',
'/mavros/imu/mag',
'/mavros/imu/static_pressure',
'/mavros/imu/temperature_baro',
'/mavros/imu/temperature_imu',
'/mavros/landing_target/lt_marker',
'/mavros/landing_target/pose_in',
'/mavros/local_position/accel',
'/mavros/local_position/odom',
'/mavros/local_position/pose',
'/mavros/local_position/pose_cov',
'/mavros/local_position/velocity_body',
'/mavros/local_position/velocity_body_cov',
'/mavros/local_position/velocity_local',
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
'/mavros/odometry/in',
'/mavros/optical_flow/ground_distance',
'/mavros/optical_flow/raw/optical_flow',
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
'/mavros/wind_estimation',
'/move_base_simple/goal',
'/uas1/mavlink_source',
'/uas1/mavlink_sink',
'/tf'
]

topics_to_adjust = ['/left/image_raw']

first=True
altosBagTimestamp = float(0)
altosMsgTimestamp = float(0)

reader = rosbag2_py.SequentialReader()
reader.open(
    rosbag2_py.StorageOptions(uri='data/new_rosbag2_2024_08_24-17_29_54', storage_id='sqlite3'),
    rosbag2_py.ConverterOptions(input_serialization_format='cdr',
                                output_serialization_format='cdr')
)

writer = rosbag2_py.SequentialWriter()
writer.open(
    rosbag2_py.StorageOptions(uri='data/final_rosbag2_2024_08_24-17_29_54', storage_id='sqlite3'),
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
    # elif topic == '/altos/altosRadar' and first:
    #     altosBagTimestamp = float(timestamp)/1e9
    #     msg_type = get_message(typename(topic))
    #     msg = deserialize_message(data, msg_type)
    #     altosMsgTimestamp = float(msg.header.stamp.sec)+float(msg.header.stamp.nanosec)/float(1e9)
    #     first=False
    # elif topic in topics_to_adjust:
    #     bag_time_offset = altosBagTimestamp - float(timestamp)/1e9
    #     msg_type = get_message(typename(topic))
    #     msg = deserialize_message(data, msg_type)
    #     if first:
    #         raise Exception("DIDNT GET ALTOS MESSAGE FIRST") 
    #     adjustedMsgTimestamp = altosMsgTimestamp - bag_time_offset
    #     msg.header.stamp.sec = int(math.floor(adjustedMsgTimestamp))
    #     msg.header.stamp.nanosec = int((adjustedMsgTimestamp % 1)*1e9)
    #     data = serialize_message(msg)
    # elif topic=='/tf':
    #     msg_type = get_message(typename(topic))
    #     msg = deserialize_message(data, msg_type)
    #     for transform in msg.transforms:
    #         if transform.header.frame_id == 'camera':
    #             filter_out = True
    #             break
        
    if not filter_out:
        writer.create_topic(
            rosbag2_py.TopicMetadata(
                name=topic, type=typename(topic), serialization_format="cdr"
            )
        )
        writer.write(topic, data, timestamp)

