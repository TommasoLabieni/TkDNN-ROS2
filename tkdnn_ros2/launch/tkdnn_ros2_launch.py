from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    ld = LaunchDescription()

    config_node = os.path.join(
        get_package_share_directory('tkdnn_ros2'),
        'config',
        'tkdnn_ros2.yaml'
        )

    node=Node(
            package='tkdnn_ros2',
            name='camera_node',
            executable='camera_node',
            parameters=[config_node]
        )

    ld.add_action(node)
    return ld