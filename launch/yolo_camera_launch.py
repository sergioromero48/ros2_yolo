from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Camera node
        Node(
            package='camera_ros',
            executable='camera_node',
            name='camera_node',
            output='screen',
            parameters=[
                {'width': 1920},
                {'height': 1080},
                {'format': 'MJPEG'}, 
            ]
        ),
        
        Node(
            package='yolo_pkg',
            executable='yolo_test',
            name='yolo_test_node',
            output='screen',
        ),
    ])
