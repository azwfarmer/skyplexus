import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    rectify_left_node = ComposableNode(
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::RectifyNode',
        name='left_rectify',
        parameters=[{
            'output_width': 1920,
            'output_height': 1080,
        }],
        remappings=[
            ('image_raw', '/camera/color/image_raw'),
            ('camera_info', '/camera/color/camera_info'),
            ('image_rect', '/camera/color/image_rect'),
            ('camera_info_rect', '/camera/color/rect_camera_info'),
        ])

    crop_node = ComposableNode(
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::CropNode',
        name='crop',
        parameters=[{
            'input_width': 1920,
            'input_height': 1080,
            'crop_width': 1080,
            'crop_height': 1080,
            'crop_mode': 'CENTER'
        }],
        remappings=[
            ('image', '/camera/color/image_rect'),
            ('camera_info', '/camera/color/rect_camera_info'),
        ])

    resize_node = ComposableNode(
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::ResizeNode',
        name='resize',
        parameters=[{
            'output_width': 224,
            'output_height': 224,
        }],
        remappings=[
            ('image', '/crop/image'),
            ('camera_info', '/crop/camera_info'),
        ])

    master_container = ComposableNodeContainer(
        package='rclcpp_components',
        name='master_container',
        namespace='',
        executable='component_container_mt',
        composable_node_descriptions=[
            rectify_left_node,
            crop_node,
            resize_node,
        ],
        output='screen'
    )

    return launch.LaunchDescription([master_container])