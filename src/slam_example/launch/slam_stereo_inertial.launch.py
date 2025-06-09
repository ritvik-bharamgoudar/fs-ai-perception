from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    slam_pkg = os.getenv("AMENT_PREFIX_PATH").split(":")[0]  # Use installed share path

    return LaunchDescription([
        Node(
            package='slam_example',
            executable='orb_slam3_stereo_inertial',
            name='orb_slam3_stereo_inertial',
            output='screen',
            parameters=[
                {
                    "vocab_path": os.path.join(slam_pkg, 'share/slam_example', 'config', 'ORBvoc.txt'),
                    "config_path": os.path.join(slam_pkg, 'share/slam_example', 'config', 'camera_and_slam_settings.yaml'),
                    "camera_frame": "zed_left_camera_optical_frame",
                    "viewer": True  # Pangolin viewer
                }
            ]
        )
    ])