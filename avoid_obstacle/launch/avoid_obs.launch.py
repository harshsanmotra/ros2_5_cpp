from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='avoid_obstacle',
            executable='avoid_obs_node',
            output='screen'),
    ])