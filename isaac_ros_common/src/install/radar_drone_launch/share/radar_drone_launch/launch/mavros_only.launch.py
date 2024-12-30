import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    mavros_apm = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(['/opt/ros/humble/share/mavros/launch/apm.launch']))

    # imu_integrator = Node(
    #     package='perception',
    #     executable = 'imu_integrator',
    #     name='imu_integrator'
    # )

    return LaunchDescription([
        mavros_apm,
        # imu_integrator
    ])
