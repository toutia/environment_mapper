##################""" install rtabmap core #############################

# install ros2 
https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html




# install g2o from source 

                sudo apt install -y cmake g++ libeigen3-dev libsuitesparse-dev
                   
                # Clone the g2o repository
                git clone https://github.com/RainerKuemmerle/g2o.git
                cd g2o

                # Create a build directory
                mkdir build && cd build
               
                # Build and install g2o  #todo ADDED CPARSe  graph optimizer 
                cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON  -DG2O_USE_CSPARSE=ON
                make -j$(nproc)
                sudo make install




# buil rtab map and rtab map ros from source to take inti acount g2o sparse solver for graph optimization 
export G2O_DIR=/usr/local



in  the repository make sure the rigth branch is used   



https://github.com/introlab/rtabmap_ros/tree/humble-devel

source ~/ros2_ws/install/setup.bash

=>   rtabmap to check if sparse solver is being used => no warning regarding this

# read this documentaton
http://wiki.ros.org/rtabmap_ros
Exemple : http://wiki.ros.org/rtabmap_slam



########################""" when relasense is not detected after installing ros rtabmap  ##########################

the rtabmap ros package have as a dependency librealsense2 package wit=ch will be prioritized over the built version (using the script / that detects the camera )

before installation 

touti@ubuntu:~/dev/media_manager$ ldd $(which realsense-viewer) | grep librealsense
	librealsense2-gl.so.2.55 => /usr/local/lib/librealsense2-gl.so.2.55 (0x0000ffffa84d0000)
	librealsense2.so.2.55 => /usr/local/lib/librealsense2.so.2.55 (0x0000ffffa7950000)



after installation 

touti@ubuntu:~/dev/media_manager$ ldd $(which realsense-viewer) | grep librealsense
	librealsense2-gl.so.2.55 => /usr/local/lib/librealsense2-gl.so.2.55 (0x0000ffff83530000)
	librealsense2.so.2.55 => /opt/ros/humble/lib/aarch64-linux-gnu/librealsense2.so.2.55 (0x0000ffff82d70000)



forcing the system library loader to prioritize the custom library built with the script situated /usr/local/lib
=>  echo 'export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH' >> ~/.bashrc
source ~/.bashrc

#################################" launching the nodes ########################################################

# launching realsense cameras 'see documentation for many cameras "



ros2 launch realsense2_camera rs_launch.py     align_depth:=true     unite_imu_method:=0     enable_gyro:=true      enable_accel:=true


ros2 run imu_filter_madgwick imu_filter_madgwick_node    --ros-args    -r imu/data_raw:=/camera/camera/accel/sample    -r imu/data:=/rtabmap/imu  -p _world_frame:="enu"  -p use_mag:=false    -p publish_tf:=false

# launch in mapping mode 

ros2 launch rtabmap_launch rtabmap.launch.py     rtabmap_args:="--delete_db_on_start --Optimizer/GravitySigma 0.3"     depth_topic:=/camera/camera/depth/image_rect_raw     rgb_topic:=/camera/camera/color/image_raw    camera_info_topic:=/camera/camera/color/camera_info     approx_sync:=false     wait_imu_to_init:=true     imu_topic:=/rtabmap/imu



####################################################   using launch file #############################################


rm -rf ~/.ros/rtabmap.db 


ros2 launch  realsense_d435i_launch.py