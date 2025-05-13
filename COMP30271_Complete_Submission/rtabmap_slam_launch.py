# Launch file for RTAB-Map SLAM
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rtabmap_ros',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[{'frame_id': 'base_link', 'subscribe_scan': True, 'use_sim_time': True}],
            remappings=[('scan', '/scan'), ('odom', '/odom'), ('rgb/image', '/image_raw'), ('rgb/camera_info', '/camera_info')]
        )
    ])
