import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('mindvision_camera'), 'config', 'camera_params.yaml')

    camera_info_url = 'package://mindvision_camera/config/camera_info_mv133.yaml'
    DeclareLaunchArgument(name='params_file',
                            default_value=params_file),
    DeclareLaunchArgument(name='camera_info_url',
                            default_value=camera_info_url),
    DeclareLaunchArgument(name='use_sensor_data_qos',
                            default_value='false'),

    container = ComposableNodeContainer(
        name='Container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='mindvision_camera',
                plugin='RMCamera::MVCamera',
                name='mindvision_camera_node',
                parameters=[LaunchConfiguration('params_file'), {
                'camera_info_url': LaunchConfiguration('camera_info_url'),
                'use_sensor_data_qos': LaunchConfiguration('use_sensor_data_qos'),
                }],
                emulate_tty=True,
            )
        ],
        output='screen'
    )
    return launch.LaunchDescription([
        container
        ])



