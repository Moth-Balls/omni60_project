from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='omni60_project',
            executable='omni60_project',
            name='omni60_node'
        ),
        Node(
            package='omni60_project',
            executable='find_center',
            name='find_center_node',
            parameters=[
                {'cluster_tolerance': 0.4},
                {'min_cluster_size': 10},
                {'max_cluster_size': 1000}
            ]
        ),
    ])