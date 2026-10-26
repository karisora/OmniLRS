"""Convenience entry point for the ROS 2 Site20 simulation."""

from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    path = Path(get_package_share_directory('lumos1_rockerless_URDF_description'))
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(path/'launch/site20.launch.py')))
    ])
