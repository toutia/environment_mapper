from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_package_prefix
import os
import sys

"""

ros2 run rviz2 rviz2 -d /home/touti/ros2_ws/install/launch/display.rviz
"""
launch_args = [
    DeclareLaunchArgument(name="namespace", default_value="", description="namespace"),
    # DeclareLaunchArgument(
    #     name="config",
    #     default_value="euroc_mav",
    #     description="euroc_mav, tum_vi, rpng_aruco...",
    # ),
    DeclareLaunchArgument(
        name="verbosity",
        default_value="ALL",
        description="ALL, DEBUG, INFO, WARNING, ERROR, SILENT",
    ),
    DeclareLaunchArgument(name="use_stereo", default_value="true", description=""),
    DeclareLaunchArgument(name="max_cameras", default_value="2", description=""),
]


def launch_setup(context):
    # configs_dir = os.path.join(get_package_share_directory("ov_msckf"), "config")
    # available_configs = os.listdir(configs_dir)
    # config = LaunchConfiguration("config").perform(context)
    # if not config in available_configs:
    #     return [
    #         LogInfo(
    #             msg="ERROR: unknown config: '{}' - Available configs are: {} - not starting OpenVINS".format(
    #                 config, ", ".join(available_configs)
    #             )
    #         )
    #     ]
    config_path = os.path.join(
        ".",
        "rs_435i",
        "estimator_config.yaml",
    )
    node1 = Node(
        package="ov_msckf",
        executable="run_subscribe_msckf",
        namespace=LaunchConfiguration("namespace"),
        parameters=[
            {"verbosity": LaunchConfiguration("verbosity")},
            {"use_stereo": LaunchConfiguration("use_stereo")},
            {"max_cameras": LaunchConfiguration("max_cameras")},
            {"config_path": config_path},
        ],
    )
    return [node1]


def generate_launch_description():
    opfunc = OpaqueFunction(function=launch_setup)
    ld = LaunchDescription(launch_args)
    ld.add_action(opfunc)
    return ld
