from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    ld.add_action(Node(package='tb4_toy',
                       executable='toy_node'))

    return ld
