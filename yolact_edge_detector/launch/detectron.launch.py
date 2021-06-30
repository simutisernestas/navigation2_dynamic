import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('yolact_edge_detector'),
        'config',
        'detector.yaml'
    )

    yolact_edge_node = Node(
        package='yolact_edge_detector',
        name='yolact_edge_node',
        executable='yolact_edge_node',
        output='screen',
        parameters=[config]
    )

    return launch.LaunchDescription([yolact_edge_node])
