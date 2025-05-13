# Launch file combining vision and traffic control
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot_vision_pkg',
            executable='vision_node',
            name='vision_node',
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
        )
    ])
