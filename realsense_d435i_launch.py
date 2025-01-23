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

def generate_launch_description():

    return LaunchDescription([

        # Launch arguments
       
        DeclareLaunchArgument(
            'unite_imu_method', default_value='2',
            description='0-None, 1-copy, 2-linear_interpolation. Use unite_imu_method:="1" if imu topics stop being published.'),

        # Make sure IR emitter is enabled
        SetParameter(name='depth_module.emitter_enabled', value=1),

        # Launch camera driver
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('realsense2_camera'), 'launch'),
                '/rs_launch.py']),
                launch_arguments={'camera_namespace': '',
                                  'camera_name': 'camera1',
                                #   'serial_no':str('042222071132'),
                                  'enable_gyro': 'true',
                                  'enable_accel': 'true',
                                  'unite_imu_method': '2',
                                  'align_depth.enable': 'true',
                                  'enable_sync': 'true',
                                  'rgb_camera.profile': '1280x720x30',
                                  'depth_module.depth_profile':'1280x720x30'}.items(),
        ),
        # Transform for camera (rotated by 45° about Y-axis)
        Node(
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=[
                '--x', '0',            # X translation
                '--y', '0',            # Y translation
                '--z', '0',            # Z translation
                '--roll', '0',         # Roll rotation
                '--pitch', '0',        # Pitch rotation
                '--yaw', '0',          # Yaw rotation
                '--frame-id', 'base_link',    # Parent frame ID
                '--child-frame-id', 'camera1_link'  # Child frame ID
            ],
        ),

     

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('realsense2_camera'), 'launch'),
                '/rs_launch.py']),
                launch_arguments={'camera_namespace': '',
                                  'camera_name': 'camera2',
                                #   'serial_no':str('036522072529'),
                                  'enable_gyro': 'true',
                                  'enable_accel': 'true',
                                  'unite_imu_method': '2',
                                  'align_depth.enable': 'true',
                                  'enable_sync': 'true',
                                  'rgb_camera.profile': '1280x720x30',
                                  'depth_module.depth_profile':'1280x720x30'}.items(),
        ),
        # Transform for camera2 (rotated by 45° about Y-axis)
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            arguments=[
                '--x', '0',            # X translation
                '--y', '0',            # Y translation
                '--z', '0',            # Z translation
                '--roll', '3.14159',         # Roll rotation
                '--pitch', '3.14159',        # Pitch rotation
                '--yaw', '0',          # Yaw rotation
                '--frame-id', 'base_link',    # Parent frame ID
                '--child-frame-id', 'camera2_link'  # Child frame ID
            ], # 0.785398 rad = 45°     stacked vertically with angle 45 degrees 
        ),
      
        Node(
            package='rtabmap_sync', executable='rgbd_sync', output='screen',namespace='camera1',name='rgbd_sync0',
            parameters=[{
                # 'frame_id':'camera_link',
                'approx_sync':True,
                'approx_sync_max_interval':0.01,
                'subscribe_rgbd':True,
                'rgbd_cameras':2,
                'wait_imu_to_init':True,
                # 'topic_queue_size' : 100,
                # 'sync_queue_size'  : 100
                }],
            remappings=[

          ('/camera1/rgb/image', '/camera1/color/image_raw'),
          ('/camera1/rgb/camera_info', '/camera1/color/camera_info'),
          ('/camera1/depth/image', '/camera1/aligned_depth_to_color/image_raw'),
    
         ]),

        Node(
            package='rtabmap_sync', executable='rgbd_sync', output='screen',namespace='camera2',name='rgbd_sync1',
            parameters=[{
                # 'frame_id':'camera_link',
                'approx_sync':True,
                'approx_sync_max_interval':0.01,
                'subscribe_rgbd':True,
                'rgbd_cameras':2,
                'wait_imu_to_init':True,
                # 'topic_queue_size' : 100,
                # 'sync_queue_size'  : 100
                }],
            remappings=[
     
          ('/camera2/rgb/image', '/camera2/color/image_raw'),
          ('/camera2/rgb/camera_info', '/camera2/color/camera_info'),
          ('/camera2/depth/image', '/camera2/aligned_depth_to_color/image_raw'),

      ]),


        Node(
        package='rtabmap_odom', executable='rgbd_odometry', output='screen',
        parameters=[{
           
            'approx_sync':True,
            'approx_sync_max_interval':0.01,
            'subscribe_rgbd':True,
            'publish_tf':True,
            'rgbd_cameras':2,
            'wait_imu_to_init':True,
             'Reg/Strategy': '1',  # Set to 1 for 3D-3D registration
             'imu_queue_size':500,
                #    'topic_queue_size' : 100,
                # 'sync_queue_size'  : 100
            }],
        remappings=[('/rgbd_image0','/camera1/rgbd_image'),
        ('/rgbd_image1','/camera2/rgbd_image'),
         ('/camera2/rgb/image', '/camera2/color/image_raw'),
        
        ('/imu','/imu/data')],# this needs to be fixed combine imus ?? 
        ),


        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[{
                
                'subscribe_rgbd':True,
                'rgbd_cameras': 2,
                'subscribe_odom_info':True,
                'approx_sync':True,
                'approx_sync_max_interval':0.01,
                'database_path':'environments/rtabmap.db',
                # 'topic_queue_size' : 100,
                # 'sync_queue_size'  : 100
               
                }],
            remappings=[('/rgbd_image0','/camera1/rgbd_image'),('/rgbd_image1','/camera2/rgbd_image')],
            arguments=['-d' ]), # this deletes the previous database

        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=[{
               
                'subscribe_rgbd':True,
                'rgbd_cameras': 2,
                'subscribe_odom_info':True,
                'approx_sync':True,
                'approx_sync_max_interval':0.01,
                'database_path':'environments/rtabmap.db',
                # 'topic_queue_size' : 100,
                # 'sync_queue_size'  : 100
               
                }],
            remappings=[('/rgbd_image0','/camera1/rgbd_image'),
        ('/rgbd_image1','/camera2/rgbd_image'),
          ('/camera2/rgb/camera_info', '/camera2/color/camera_info'),
          ('/camera2/depth/image', '/camera2/aligned_depth_to_color/image_raw'),
        ('/imu','/imu/data')],# this needs to be fixed combine imus ?? 
       ),

        # Compute quaternion of the IMU
        Node(
            package='imu_filter_madgwick', executable='imu_filter_madgwick_node', output='screen',
            parameters=[{'use_mag': False, 
                         'world_frame':'enu', 
                         'publish_tf':False}],
            remappings=[('imu/data_raw', '/camera1/imu')]),
    ])