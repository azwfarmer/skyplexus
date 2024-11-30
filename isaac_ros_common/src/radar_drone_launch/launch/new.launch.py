import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node
import launch
import numpy as np
from scipy.spatial.transform import Rotation as R

def generate_launch_description():
    # t_ekf2lidar = np.array([0,0,-1])
    # r_ekf2lidar = R.from_euler('y',160, degrees=True)
    # q_ekf2lidar = r_ekf2lidar.as_quat()

    # t_lidar2ekf = np.array([0,0,-1])
    # r_lidar2ekf = R.from_euler('y',-160, degrees=True)
    # q_lidar2ekf = r_lidar2ekf.as_quat()

    # t_lidar2radar = np.array([0.4, 0.4, 1.4])
    # r_lidar2radar = R.from_euler('y', 180, degrees=True)

    # t_radar2camera = np.array([0,0,0.15])

    # t_ekf2radar = t_ekf2lidar + r_ekf2lidar.apply(t_lidar2radar)
    # r_ekf2radar = r_ekf2lidar * r_lidar2radar
    # q_ekf2radar = r_ekf2radar.as_quat()

    # t_ekf2camera = t_ekf2radar + r_ekf2radar.apply(t_radar2camera)
    # q_ekf2camera = q_ekf2radar

    jetson_stats = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(['/opt/ros/humble/share/isaac_ros_jetson_stats/launch/jtop.launch.py']))

    orbbec_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(['/workspaces/isaac_ros-dev/isaac_ros_common/src/OrbbecSDK_ROS2/orbbec_camera/launch/astra2.launch.py'])
    )

    mavros_apm = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(['/opt/ros/humble/share/mavros/launch/apm.launch']),
        launch_arguments={
            'log_output': 'log'
        }.items()
    )

    ouster_driver = IncludeLaunchDescription(
        # PythonLaunchDescriptionSource(['/workspaces/isaac_ros-dev/isaac_ros_common/ouster-ros/ouster-ros/launch/sensor.composite.launch.py']),
        XMLLaunchDescriptionSource(['/workspaces/isaac_ros-dev/isaac_ros_common/src/ouster-ros/ouster-ros/launch/sensor.launch.xml']),
        launch_arguments={
                'sensor_hostname': '192.168.0.100', 
                'viz': 'false',
                'udp_profile_lidar': 'RNG19_RFL8_SIG16_NIR16',
                'udp_dest': '192.168.0.99',
                'lidar_mode': '1024x20',
                'timestamp_mode': 'TIME_FROM_PTP_1588', #,'TIME_FROM_INTERNAL_OSC'
                'proc_mask': 'PCL|IMU', #IMG|PCL|IMU|SCAN
        }.items()
    )

    altos_parse = Node(
        package = 'altosradar',
        namespace = 'altos',
        executable = 'altosRadarParse',
        name='altos'
    )

    imu_integrator = Node(
        package='perception',
        executable = 'imu_integrator',
        name='imu_integrator'
    )

    overlay = Node(
        package='perception',
        executable = 'perception',
        name='overlay'
    )

    base_link2payload = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='os_base_static_tf_pub',
        arguments=["0","0","0",
            "0","0","0",
            'base_link','payload']
    )

    payload2os_sensor = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='os_base_static_tf_pub',
        arguments=["0","0","0",
            "0","0","0",
            'payload','os_sensor']
    )

    payload2radar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='os_base_static_tf_pub',
        arguments=["0","0","1",
            "0","0","0",
            'payload','altosRadar']
    )

    payload2camera_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='os_base_static_tf_pub',
        arguments=["0","0","2",
            "0","0","0",
            'payload','camera_link']
    )

    everything_recorder = launch.actions.ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-a', '-x', '(.*)theora(.*)|(.*)compressed(.*)'],
        output='both'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', '/workspaces/isaac_ros-dev/isaac_ros_common/src/rviz_camlidrad.rviz']
    )

    return LaunchDescription([
        jetson_stats,
        orbbec_camera,
        mavros_apm,
        ouster_driver,
        altos_parse,
        base_link2payload,
        payload2os_sensor,
        payload2radar,
        payload2camera_link,
        #imu_integrator,
        everything_recorder,
        rviz
    ])
