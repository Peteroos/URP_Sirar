"""
Launch file for ArUco marker detection node.

Usage:
    ros2 launch aruco_detector aruco_detect.launch.py
    ros2 launch aruco_detector aruco_detect.launch.py target_id:=27 marker_length_m:=0.19
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('target_id', default_value='27',
                              description='ArUco marker ID to detect'),
        DeclareLaunchArgument('marker_length_m', default_value='0.19',
                              description='Physical marker side length in meters'),
        DeclareLaunchArgument('pose_method', default_value='aruco_pnp',
                              description='Pose estimation method: aruco_pnp or depth_center'),
        DeclareLaunchArgument('publish_coordinate_convention', default_value='optical',
                              description='Output coordinate convention: optical or camera_link'),
        DeclareLaunchArgument('publish_tf_frame', default_value='true',
                              description='Whether to broadcast marker TF frame'),
        DeclareLaunchArgument('marker_frame_name', default_value='aruco_tag',
                              description='TF frame name for the detected marker'),

        Node(
            package='aruco_detector',
            executable='detect_vis',
            name='aruco_detector_node',
            output='screen',
            parameters=[{
                'target_id': LaunchConfiguration('target_id'),
                'marker_length_m': LaunchConfiguration('marker_length_m'),
                'pose_method': LaunchConfiguration('pose_method'),
                'publish_coordinate_convention': LaunchConfiguration('publish_coordinate_convention'),
                'publish_tf_frame': LaunchConfiguration('publish_tf_frame'),
                'marker_frame_name': LaunchConfiguration('marker_frame_name'),
            }],
        ),
    ])
