# Requirements:
#   A realsense D435i
#   Install realsense2 ros2 package (ros-$ROS_DISTRO-realsense2-camera)
# Example:
#   $ ros2 launch rtabmap_examples realsense_d435i_color.launch.py

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction


configurable_parameters = [
    {
        "name": "usb_port_id",
        "default": "''",
        "description": "choose device by usb port id",
    },
    {"name": "device_type", "default": "''", "description": "choose device by type"},
    {"name": "config_file", "default": "''", "description": "yaml config file"},
    {
        "name": "json_file_path",
        "default": "''",
        "description": "allows advanced configuration",
    },
    {"name": "initial_reset", "default": "false", "description": "''"},
    {
        "name": "accelerate_gpu_with_glsl",
        "default": "false",
        "description": "enable GPU acceleration with GLSL",
    },
    {
        "name": "rosbag_filename",
        "default": "''",
        "description": "A realsense bagfile to run from as a device",
    },
    {
        "name": "log_level",
        "default": "info",
        "description": "debug log level [DEBUG|INFO|WARN|ERROR|FATAL]",
    },
    {
        "name": "output",
        "default": "screen",
        "description": "pipe node output [screen|log]",
    },
    {"name": "enable_color", "default": "true", "description": "enable color stream"},
    {
        "name": "rgb_camera.color_profile",
        "default": "848,480,60",
        "description": "color stream profile",
    },
    {
        "name": "rgb_camera.color_format",
        "default": "RGB8",
        "description": "color stream format",
    },
    {
        "name": "rgb_camera.enable_auto_exposure",
        "default": "true",
        "description": "enable/disable auto exposure for color image",
    },
    {"name": "enable_depth", "default": "true", "description": "enable depth stream"},
    {
        "name": "enable_infra",
        "default": "false",
        "description": "enable infra60 stream",
    },
    {
        "name": "enable_infra1",
        "default": "false",
        "description": "enable infra1 stream",
    },
    {
        "name": "enable_infra2",
        "default": "false",
        "description": "enable infra2 stream",
    },
    {
        "name": "depth_module.depth_profile",
        "default": "848,480,60",
        "description": "depth stream profile",
    },
    {
        "name": "depth_module.depth_format",
        "default": "Z16",
        "description": "depth stream format",
    },
    {
        "name": "depth_module.infra_profile",
        "default": "0,0,0",
        "description": "infra streams (0/1/2) profile",
    },
    {
        "name": "depth_module.infra_format",
        "default": "RGB8",
        "description": "infra0 stream format",
    },
    {
        "name": "depth_module.infra1_format",
        "default": "Y8",
        "description": "infra1 stream format",
    },
    {
        "name": "depth_module.infra2_format",
        "default": "Y8",
        "description": "infra2 stream format",
    },
    {
        "name": "depth_module.exposure",
        "default": "8500",
        "description": "Depth module manual exposure value",
    },
    {
        "name": "depth_module.gain",
        "default": "16",
        "description": "Depth module manual gain value",
    },
    {
        "name": "depth_module.hdr_enabled",
        "default": "false",
        "description": "Depth module hdr enablement flag. Used for hdr_merge filter",
    },
    {
        "name": "depth_module.enable_auto_exposure",
        "default": "true",
        "description": "enable/disable auto exposure for depth image",
    },
    {
        "name": "depth_module.exposure.1",
        "default": "7500",
        "description": "Depth module first exposure value. Used for hdr_merge filter",
    },
    {
        "name": "depth_module.gain.1",
        "default": "16",
        "description": "Depth module first gain value. Used for hdr_merge filter",
    },
    {
        "name": "depth_module.exposure.2",
        "default": "1",
        "description": "Depth module second exposure value. Used for hdr_merge filter",
    },
    {
        "name": "depth_module.gain.2",
        "default": "16",
        "description": "Depth module second gain value. Used for hdr_merge filter",
    },
    {"name": "enable_sync", "default": "true", "description": "'enable sync mode'"},
    {"name": "enable_rgbd", "default": "false", "description": "'enable rgbd topic'"},
    {"name": "enable_gyro", "default": "true", "description": "'enable gyro stream'"},
    {
        "name": "enable_accel",
        "default": "true",
        "description": "'enable accel stream'",
    },
    {"name": "gyro_fps", "default": "0", "description": "''"},
    {"name": "accel_fps", "default": "0", "description": "''"},
    {
        "name": "unite_imu_method",
        "default": "2",
        "description": "[0-None, 1-copy, 2-linear_interpolation]",
    },
    {"name": "clip_distance", "default": "-2.", "description": "''"},
    {"name": "angular_velocity_cov", "default": "0.01", "description": "''"},
    {"name": "linear_accel_cov", "default": "0.01", "description": "''"},
    {
        "name": "diagnostics_period",
        "default": "0.0",
        "description": "Rate of publishing diagnostics. 0=Disabled",
    },
    {
        "name": "publish_tf",
        "default": "true",
        "description": "[bool] enable/disable publishing static & dynamic TF",
    },
    {
        "name": "tf_publish_rate",
        "default": "0.0",
        "description": "[double] rate in Hz for publishing dynamic TF",
    },
    {"name": "pointcloud.enable", "default": "false", "description": ""},
    {
        "name": "pointcloud.stream_filter",
        "default": "2",
        "description": "texture stream for pointcloud",
    },
    {
        "name": "pointcloud.stream_index_filter",
        "default": "0",
        "description": "texture stream index for pointcloud",
    },
    {"name": "pointcloud.ordered_pc", "default": "false", "description": ""},
    {
        "name": "pointcloud.allow_no_texture_points",
        "default": "false",
        "description": "''",
    },
    {
        "name": "align_depth.enable",
        "default": "true",
        "description": "enable align depth filter",
    },
    {
        "name": "colorizer.enable",
        "default": "false",
        "description": "enable colorizer filter",
    },
    {
        "name": "decimation_filter.enable",
        "default": "false",
        "description": "enable_decimation_filter",
    },
    {
        "name": "spatial_filter.enable",
        "default": "false",
        "description": "enable_spatial_filter",
    },
    {
        "name": "temporal_filter.enable",
        "default": "false",
        "description": "enable_temporal_filter",
    },
    {
        "name": "disparity_filter.enable",
        "default": "false",
        "description": "enable_disparity_filter",
    },
    {
        "name": "hole_filling_filter.enable",
        "default": "false",
        "description": "enable_hole_filling_filter",
    },
    {
        "name": "hdr_merge.enable",
        "default": "false",
        "description": "hdr_merge filter enablement flag",
    },
    {
        "name": "wait_for_device_timeout",
        "default": "-1.",
        "description": "Timeout for waiting for device to connect (Seconds)",
    },
    {
        "name": "reconnect_timeout",
        "default": "2.",
        "description": "Timeout(seconds) between consequtive reconnection attempts",
    },
]


def declare_configurable_parameters(parameters):
    return [
        DeclareLaunchArgument(
            param["name"],
            default_value=param["default"],
            description=param["description"],
        )
        for param in parameters
    ]


def set_configurable_parameters(parameters):
    return dict(
        [(param["name"], LaunchConfiguration(param["name"])) for param in parameters]
    )


def generate_launch_description():

    return LaunchDescription(
        # Launch arguments*
        declare_configurable_parameters(configurable_parameters)
        + [SetParameter(name="depth_module.emitter_enabled", value=1)]
        + [
            Node(
                package="realsense2_camera",
                executable="realsense2_camera_node",
                namespace="",
                name="camera1",
                output="screen",
                parameters=[set_configurable_parameters(configurable_parameters)],
                arguments=[
                    "--ros-args",
                    "--log-level",
                    "info",
                    "-p",
                    "serial_no:='036522072529'",
                    "-p",
                    "camera_name:='camera1'",
                ],  # Specify the serial number
                emulate_tty=True,
            ),
            # Transform for camera (rotated by 45° about Y-axis)
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                output="screen",
                arguments=[
                    "--x",
                    "0",  # X translation
                    "--y",
                    "0",  # Y translation
                    "--z",
                    "0",  # Z translation
                    "--roll",
                    "0",  # Roll rotation
                    "--pitch",
                    "0",  # Pitch rotation
                    "--yaw",
                    "0",  # Yaw rotation
                    "--frame-id",
                    "base_link",  # Parent frame ID
                    "--child-frame-id",
                    "camera1_link",  # Child frame ID
                ],
            ),
            Node(
                package="realsense2_camera",
                executable="realsense2_camera_node",
                namespace="",
                name="camera2",
                parameters=[set_configurable_parameters(configurable_parameters)],
                output="screen",
                arguments=[
                    "--ros-args",
                    "--log-level",
                    "info",
                    "-p",
                    "serial_no:='042222071132'",
                    "-p",
                    "camera_name:='camera2'",
                ],  # Specify the serial number
                emulate_tty=True,
            ),
            # Transform for camera2 (rotated by 45° about Y-axis)
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=[
                    "--x",
                    "-0.018",  # X translation
                    "--y",
                    "0",  # Y translation
                    "--z",
                    "-0.071",  # Z translation
                    "--roll",
                    "0",  # -0.279 Roll rotation
                    "--pitch",
                    "0.4",  # Pitch rotation
                    "--yaw",
                    "0",  # Yaw rotation
                    "--frame-id",
                    "base_link",  # Parent frame ID
                    "--child-frame-id",
                    "camera2_link",  # Child frame ID
                ],
                emulate_tty=True,
            ),
            Node(
                package="rtabmap_sync",
                executable="rgbd_sync",
                output="screen",
                namespace="camera1",
                name="rgbd_sync0",
                parameters=[
                    {
                        # 'frame_id':'camera_link',
                        "approx_sync": True,
                        "approx_sync_max_interval": 0.1,
                        "subscribe_rgbd": True,
                        "rgbd_cameras": 2,
                        "wait_imu_to_init": True,
                        "topic_queue_size": 10,
                        "sync_queue_size": 10,
                    }
                ],
                remappings=[
                    ("/camera1/rgb/image", "/camera1/color/image_raw"),
                    ("/camera1/rgb/camera_info", "/camera1/color/camera_info"),
                    (
                        "/camera1/depth/image",
                        "/camera1/aligned_depth_to_color/image_raw",
                    ),
                ],
            ),
            Node(
                package="rtabmap_sync",
                executable="rgbd_sync",
                output="screen",
                namespace="camera2",
                name="rgbd_sync1",
                parameters=[
                    {
                        # 'frame_id':'camera_link',
                        "approx_sync": True,
                        "approx_sync_max_interval": 0.1,
                        "subscribe_rgbd": True,
                        "rgbd_cameras": 2,
                        "wait_imu_to_init": True,
                        "topic_queue_size": 10,
                        "sync_queue_size": 10,
                    }
                ],
                remappings=[
                    ("/camera2/rgb/image", "/camera2/color/image_raw"),
                    ("/camera2/rgb/camera_info", "/camera2/color/camera_info"),
                    (
                        "/camera2/depth/image",
                        "/camera2/aligned_depth_to_color/image_raw",
                    ),
                ],
            ),
            Node(
                package="rtabmap_sync",
                executable="rgbdx_sync",
                output="screen",
                namespace="cameras",
                name="rgbdx_sync",
                parameters=[
                    {
                        "approx_sync": True,
                        "approx_sync_max_interval": 0.1,
                        "rgbd_cameras": 2,
                    }
                ],
                remappings=[
                    ("rgbd_image0", "/camera1/rgbd_image"),
                    ("rgbd_image1", "/camera2/rgbd_image"),
                ],
            ),
            Node(
                package="imu_filter_madgwick",
                executable="imu_filter_madgwick_node",
                output="screen",
                parameters=[
                    {
                        "use_mag": False,
                        "world_frame": "enu",
                        "publish_tf": False,
                        "topic_queue_size": 10,
                        "sync_queue_size": 10,
                    }
                ],
                remappings=[("imu/data_raw", "/camera1/imu")],
            ),
            Node(
                package="rtabmap_odom",
                executable="rgbd_odometry",
                output="screen",
                parameters=[
                    {
                        "approx_sync": True,
                        "approx_sync_max_interval": 0.05,
                        "subscribe_rgbd": True,
                        "publish_tf": True,
                        "rgbd_cameras": 0,
                        "wait_imu_to_init": True,
                        "topic_queue_size": 100,
                        "sync_queue_size": 100,
                        "odom_frame_id": "odom",
                        "publish_tf": True,
                        # [0=SURF 1=SIFT 2=ORB 3=FAST/FREAK 4=FAST/BRIEF 5=GFTT/FREAK 6=GFTT/BRIEF 7=BRISK 8=GFTT/ORB 9=KAZE 10=ORB-OCTREE 11=SuperPoint 12=SURF/FREAK 13=GFTT/DAISY 14=SURF/DAISY 15=PyDetector]
                        "Vis/FeatureType": "10",
                        "Vis/MaxFeatures": "1500",  # Reduce from default 2000
                        "Vis/MinDepth": "0.3",  # Avoid noise in close range
                        "Vis/MaxDepth": "5.0",  # Ignore distant keypoints
                        "Vis/ShowKeypoints": "true",
                        "Vis/MinInliers": "10",
                        # [0=Frame-to-Map (F2M) 1=Frame-to-Frame (F2F) 2=Fovis 3=viso2 4=DVO-SLAM 5=ORB_SLAM2 6=OKVIS 7=LOAM 8=MSCKF_VIO 9=VINS-Fusion 10=OpenVINS 11=FLOAM 12=Open3D]
                        "Odom/Strategy": "1",
                        "Odom/GuessMotion": "false",
                        "Odom/KeyFrameThr": "0.3",
                        "ORB/Gpu": "true",
                    }
                ],
                remappings=[
                    ("rgbd_images", "/cameras/rgbd_images"),
                    ("/imu", "/imu/data"),
                ],  # this needs to be fixed combine imus ??
            ),
            Node(
                package="rtabmap_slam",
                executable="rtabmap",
                output="screen",
                parameters=[
                    {
                        "subscribe_rgbd": True,
                        "rgbd_cameras": 0,
                        "subscribe_odom_info": True,
                        "subscribe_odom": True,
                        "approx_sync": True,
                        "approx_sync_max_interval": 0.05,
                        "database_path": "environments/rtabmap.db",
                        "topic_queue_size": 100,
                        "sync_queue_size": 100,
                        "Rtabmap/DetectionRate": "3",
                        # six degrees of freedom roll and pitch also can
                        "RGBD/ForceOdom3DoF": "false",
                        # keypoint strategy without incidence if reusing features from odom
                        "Kp/DetectorStrategy": "8",
                        "Kp/NNStrategy": "4",
                        "Kp/ByteToFloat": "true",
                        "Kp/DescriptorCaching": "true",
                        # publish rgbd odometry
                        "RGBD/PublishOdometry": True,
                        "RGBD/OptimizeFromGraphEnd": "true",
                        "RGBD/Force2D": "false",
                        # odometry  feature REUSE  for loop closure dteetction = odometry computes keypoiçnts and features ...
                        "Mem/UseOdomFeatures": "true",
                        # Allow New Map Creation When Odometry Fails
                        "Rtabmap/CreateNewMapOnFailure": "true",
                        # Keep Past Maps and Allow Merging
                        "Rtabmap/StartNewMapOnLoopClosureDetection": "true",
                        "Mem/DetectSimilar": "true",
                        # Enable Loop Closures for Automatic Map Linking
                        "Rtabmap/DetectMoreLoopClosures": "true",
                        "Rtabmap/LoopClosureHypothesisRatio": "0.2",
                        # Prevent Data Loss & Optimize Memory
                        "Mem/NotLinkedNodesKept": "true",
                        "Mem/STMSize": "100",
                        "Mem/RehearsalSimilarity": "0.3",
                        "Odom/ResetCountdown": "50",
                        # update rgbd_cameras = 0 then map /rgbd_images to cameras/rgbd_images
                    }
                ],
                remappings=[
                    ("rgbd_images", "/cameras/rgbd_images"),
                    ("/imu", "/imu/data"),
                ],
                arguments=["-d", "--log-level", "debug", "--delete_db_on_start"],
            ),  # this deletes the previous database
            Node(
                package="rtabmap_viz",
                executable="rtabmap_viz",
                output="screen",
                parameters=[
                    {
                        "subscribe_rgbd": True,
                        "rgbd_cameras": 0,
                        "subscribe_odom_info": True,
                        "subscribe_odom": True,
                        "odom_frame_id": "odom",
                        "approx_sync": True,
                        "approx_sync_max_interval": 0.05,
                        "database_path": "environments/rtabmap.db",
                        "topic_queue_size": 100,
                        "sync_queue_size": 100,
                    }
                ],
                remappings=[
                    ("rgbd_images", "/cameras/rgbd_images"),
                ],  # this needs to be fixed combine imus ??
            ),
            # # Compute quaternion of the IMU
        ]
    )
