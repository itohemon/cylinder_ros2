from os.path import join

from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    apriltag_ros_pkg_prefix = get_package_share_directory('apriltag_ros')
    apriltag_launch = join(apriltag_ros_pkg_prefix,
                           'launch/tag_36h11_all.launch.py')
    apriltag_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(apriltag_launch))

    v4l2_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        output='screen'
    )

    image_transport = Node(
        package='image_transport',
        executable='republish',
        arguments=["raw"],
        remappings=[('in', '/image_raw'),
                    ('out', '/image_raw/compressed')
                   ]
    )

    return LaunchDescription([
        apriltag_node,
        v4l2_node,
        image_transport
    ])
