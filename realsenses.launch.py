import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import TimerAction, OpaqueFunction, PushLaunchConfigurations, PopLaunchConfigurations, DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetRemap
import launch_testing
import launch_testing.actions
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):

    return [
        GroupAction(actions=[
        SetRemap(
            src=LaunchConfiguration('namespace').perform(context) + '/pose/sample',
            dst=LaunchConfiguration('namespace').perform(context) + '/odom'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory("realsense2_camera"),
                             "launch", "rs_launch.py")),
                             
            launch_arguments={
                "align_depth.enable": "true",
                "enable_sync": "true",
                "rgb_camera.power_line_frequency": "2",  # Set to 50Hz or 60Hz
                "enable_depth": "true",  # Example of enabling depth stream
                "enable_color": "true",
                "log_level": "warn",
                "rgb_camera.profile": "640,480,15",
                "depth_module.depth_profile": "640,480,15",
                "camera_name": LaunchConfiguration('namespace').perform(context)[1:],
                "device_type": "d435i",
            }.items(),
            ),
            SetRemap(
            src='/camera/color/image_raw', 
            dst=LaunchConfiguration('namespace').perform(context) + 'color/image_raw'),
            SetRemap(
            src='/camera/color/camera_info', 
            dst=LaunchConfiguration('namespace').perform(context) + 'color/camera_info'),
        ])
    ]


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='/r0',
                              description=''),
        OpaqueFunction(function=launch_setup)
        ])
