from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    root = "package://mindvision_camera/config/camera_calibration.xml"

    return LaunchDescription([
        DeclareLaunchArgument(name='t',
                                default_value='mindvision_camera'),
        DeclareLaunchArgument(name='root',
                                default_value=root),
        DeclareLaunchArgument(name='frequency',
                                default_value=10000),
        Node(
            package='mindvision_camera',
            executable='mindvision_camera_node',
            parameters=[{
                "root": LaunchConfiguration('root'),
                "t": LaunchConfiguration('t'),
                "frequency": LaunchConfiguration('frequency'),
            }]
        )
    ])
