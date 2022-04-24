from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    obstacle_avoidance = Node(
        package='cylinder_ros2',
        executable='obstacle_avoidance',
        output='screen',
        remappings=[
            ('laser_scan', '/scan')
            ]
    )

    return LaunchDescription([
        obstacle_avoidance
    ])

