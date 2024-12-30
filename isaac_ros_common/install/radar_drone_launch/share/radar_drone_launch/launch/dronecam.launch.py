import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    
    argus_camera = IncludeLaunchDescription(PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('isaac_ros_argus_camera'), 'launch'),
         '/isaac_ros_argus_camera_mono.launch.py']))
    
    mavros_apm = IncludeLaunchDescription(
       XMLLaunchDescriptionSource(['/opt/ros/humble/share/mavros/launch/apm.launch']))

    imu_integrator = Node(
       package='perception',
       executable = 'imu_integrator',
       name='imu_integrator'
    )

    return LaunchDescription([
        #static_transform_publisher,
        argus_camera,
        mavros_apm,
        imu_integrator
    ])
