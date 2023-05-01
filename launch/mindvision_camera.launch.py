from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import os
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch

def generate_launch_description():
    params_file = os.path.join(get_package_share_directory('mindvision_camera'), 'config', 'camera_params.yaml')

    camera_info_url = 'package://mindvision_camera/config/camera_info_mv133_infantry3.yaml'

    return launch.LaunchDescription([
        DeclareLaunchArgument(name='params_file',
                              default_value=params_file),
        DeclareLaunchArgument(name='camera_info_url',
                              default_value=camera_info_url),
        DeclareLaunchArgument(name='use_sensor_data_qos',
                              default_value='false'),
        Node(
            package='mindvision_camera',
            executable='mindvision_camera_node',
            output='screen',
            emulate_tty=True,
            parameters=[LaunchConfiguration('params_file'), {
                'camera_info_url': LaunchConfiguration('camera_info_url'),
                'use_sensor_data_qos': LaunchConfiguration('use_sensor_data_qos'),
            }],
        ),
    ])
