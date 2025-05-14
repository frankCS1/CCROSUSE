from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot_vision_pkg',
            executable='wall_follow_node',
            name='wall_follow_node',
            output='screen'
        ),
        Node(
            package='my_robot_vision_pkg',
            executable='goal_node',
            name='goal_node',
            output='screen'
        ),
        Node(
            package='my_robot_vision_pkg',
            executable='nav_node',
            name='nav_node',
            output='screen'
        ),
        Node(
            package='my_robot_vision_pkg',
            executable='traffic_node',
            name='traffic_node',
            output='screen'
        ),
        Node(
            package='my_robot_vision_pkg',
            executable='control_node',
            name='control_node',
            output='screen'
        ),
        Node(
            package='my_robot_vision_pkg',
            executable='landmark_logger',
            name='landmark_logger',
            output='screen'
        )
    ])
