from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='mindvision_camera',
            executable='mindvision_camera_node',
        )
    ])
