from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='kaz_j25_ajr', executable='world_node', name='world'),
        Node(package='kaz_j25_ajr', executable='vacuum_node', name='vacuum')
    ])
