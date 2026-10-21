import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node
import launch
import launch_ros.actions
import numpy as np

def generate_launch_description():
    pkg_share = get_package_share_directory('radar_drone_launch')

    orbbec_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('orbbec_camera'), 'launch', 'gemini_330_series.launch.py')])
    )

    mavros_apm = IncludeLaunchDescription(
        XMLLaunchDescriptionSource([os.path.join(pkg_share, 'launch', 'mavros', 'apm.launch')]),
    )

    ouster_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ouster_ros'), 'launch', 'driver.launch.py')]),
        launch_arguments={
            'params_file': os.path.join(pkg_share, 'config', 'ouster_driver_params.yaml'),
            'ouster_ns': 'ouster',
            'os_driver_name': 'os_driver',
            'viz': 'False',
        }.items()
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
        arguments=['-d', os.path.join(pkg_share, 'config', 'rviz_camlidrad.rviz')]
    )
    
    base_link2payload = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='os_base_static_tf_pub',
        arguments=['--x', '0.0', '--y', '0.0', '--z', '0.0',
            '--roll', '0.0', '--pitch', '0.0','--yaw', '0.0',
            '--frame-id', 'base_link', '--child-frame-id', 'payload']
    )

    payload2os_sensor = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='os_base_static_tf_pub',
        arguments=['--x', '0.0', '--y', '0.0', '--z', '0.0',
            '--roll', str(np.pi), '--pitch', '0.0','--yaw', '0.0',
            '--frame-id', 'payload', '--child-frame-id', 'os_sensor']
    )

    payload2radar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='os_base_static_tf_pub',
        arguments=['--x', '0.11', '--y', '0.008', '--z', '0.116',
            '--roll', '0.0', '--pitch', '0.0','--yaw', '0.0',
            '--frame-id', 'payload', '--child-frame-id', 'radar']
    )

    payload2camera_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='os_base_static_tf_pub',
        arguments=["0.127","0.03","0.197",
            "0","0","0",
            'payload','camera_link']
    )
    
    return LaunchDescription([
        base_link2payload,
        payload2os_sensor,
        payload2radar,
        payload2camera_link,
        orbbec_camera,
        mavros_apm,
        ouster_driver,
        everything_recorder,
        rviz
    ])
