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





##############################"" inspect frames ################################################################################################




 ros2 run tf2_tools view_frames


######################################"" CUDA ENABLED  #########################################################################


# opencv with cude 
sudo apt update
sudo apt install build-essential cmake git unzip pkg-config
sudo apt install libjpeg-dev libpng-dev libtiff-dev
sudo apt install libavcodec-dev libavformat-dev libswscale-dev libv4l-dev
sudo apt install libxvidcore-dev libx264-dev libgtk-3-dev
sudo apt install libatlas-base-dev gfortran python3-dev




cd ~
git clone https://github.com/opencv/opencv.git
git clone https://github.com/opencv/opencv_contrib.git
cd opencv
mkdir build && cd build
cmake -D CMAKE_BUILD_TYPE=Release \
      -D CMAKE_INSTALL_PREFIX=/usr/local \
      -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
      -D WITH_CUDA=ON \
      -D ENABLE_FAST_MATH=1 \
      -D CUDA_FAST_MATH=1 \
      -D WITH_CUBLAS=1 \
      -D WITH_CUDNN=ON \
      -D OPENCV_DNN_CUDA=ON \
      -D WITH_TBB=ON \
      -D BUILD_opencv_cudacodec=ON \
      -D OPENCV_GENERATE_PKGCONFIG=ON ..
      ..
make -j$(nproc)
sudo make install


python3 -c "import cv2; print(cv2.getBuildInformation())" | grep -i cuda

If CUDA modules appear (e.g., cudaarithm, cudafeatures2d, cudaimgproc), it means cv_bridge is now using your CUDA-enabled OpenCV.

# cv_bridge needs to be built with cuda opencv 

cd ~/ros2_ws/src
git clone -b humble https://github.com/ros-perception/vision_opencv.git


cd ~/ros2_ws

colcon build ......see below 

# rtabmap with cuda and cudasift 


cd ~
git clone https://github.com/introlab/rtabmap.git
cd rtabmap
mkdir build && cd build
cmake -DWITH_CUDA=ON -DWITH_CUDASIFT=ON -DWITH_FASTCV=ON ..
make -j$(nproc)
sudo make install



# build opengv 
git clone https://github.com/laurentkneip/opengv.git

mkdir opengv/build
cd opengv/build
cmake ..
make
sudo make install

# Rebuild RTAB-Map with OpenGV
export G2O_DIR=/usr/local
export OpenCV_DIR=/usr/local/lib/cmake/opencv4   
export MAKEFLAGS="-j6"
https://github.com/introlab/rtabmap_ros/tree/humble-devel

   
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release   -DWITH_OPENGV=ON   -DCMAKE_BUILD_TYPE=Debug      -DWITH_TORCH=ON   -DWITH_ORB_SLAM=ON -DOpenCV_DIR=/usr/local/lib/cmake/opencv4     -DWITH_CUDASIFT=ON





# Enable GPU Feature Matching in RTAB-Map

<param name="Kp/DetectorStrategy" value="6"/>  <!-- Enable ORB GPU -->
<param name="Vis/FeatureDetectorStrategy" value="6"/>
<param name="RGBD/OptimizeMaxError" value="3.0"/>
<param name="Odom/GuessMotion" value="true"/>


# enable gpu based depth processing 
enable_depth:=true
<param name="RGBD/DepthRegistration" value="true"/>


# verification 
rtabmap --version
Check for:

with opentorch : true 

with OPenGV  true

############################################################" all cmake options to use with colcon ###################################################"

cd ~/ros2_ws/src/rtabmap
cmake . -L

# build pcl with cuda 



git clone https://github.com/PointCloudLibrary/pcl.git
cd pcl
mkdir build && cd build
cmake ..  -DCMAKE_BUILD_TYPE=Release          -DBUILD_GPU=ON                  -DBUILD_CUDA=ON         -DWITH_CUDA=ON          -DBUILD_cuda_io=ON          -DBUILD_cuda_apps=ON          -DBUILD_apps=ON          -DBUILD_simulation=ON

make -j$(nproc)
sudo make install


pcl_gpu_icp -h



# rebuild rtabmap with pcl build with cuda support 

udate cmakelists.txt for rtapmap_util conversionss  rtabmap_slam  and  odom ..
find_package(Boost REQUIRED COMPONENTS date_time)

colcon build  --cmake-args -DCMAKE_BUILD_TYPE=Release   -DWITH_OPENGV=ON         -DWITH_OPENVINS=ON  -DOpenCV_DIR=/usr/local/lib/cmake/opencv4     -DWITH_CUDASIFT=ON   


relasense


# TODO next : feature extraction using DNN 
odmometry strategy 
0 = F2M (Frame-to-Map) [Default]	Matches features against a global map	⭐⭐⭐	🟠 Slow
1 = F2F (Frame-to-Frame)	Matches features between consecutive frames	⭐⭐	🟢 Fast
5 = ORB_SLAM2	Feature-based SLAM using ORB descriptors	⭐⭐⭐⭐	🟡 Moderate
6 = OKVIS	VIO using feature tracking + IMU	⭐⭐⭐⭐	🔴 Slow
8 = MSCKF_VIO	Lightweight VIO using feature tracking + IMU	⭐⭐⭐	🟢 Fast
9 = VINS-Fusion	Factor graph-based VIO (high accuracy)	⭐⭐⭐⭐	🟡 Moderate
10 = OpenVINS	Optimization-based VIO	⭐⭐⭐⭐	🟡 Moderate


###################build okvis #####################################

https://github.com/ethz-asl/okvis
cmake -DCMAKE_BUILD_TYPE=Release .. -DCXSPARSE_INCLUDE_DIR=/usr/include/suitesparse


remove any reference to neon 
add -DUSE_SYSTEM_BRISK=ON
-DUSE_SYSTEM_CERES

require minimum cmake 3.10

############################  kalibr docker ################################
use this FROM arm64v8/ros:noetic-ros-base 
in the profided docker file (kalibr source code )



apt-get install -y ros-noetic-realsense2-camera
apt-get install -y ros-noetic-rqt-image-view

apt install vim 



docker build -t kalibr -f Dockerfile_ros1_20_04 . # change this to whatever ubuntu version you want

FOLDER=~/data/
xhost +local:root => NO DISPLAY IS AVAILABLE INSIDE CONTAINER IF THIS IS NOT EXECUTED 
sudo docker run -it -e "DISPLAY" -e "QT_X11_NO_MITSHM=1"  --device=/dev/bus/usb --network host    -v "/tmp/.X11-unix:/tmp/.X11-unix:rw"     -v "$FOLDER:/data" kalibr   



sudo docker exec -it 84d42bb12d44 bash  => source setup.bash 
roslaunch /data/rs_d435i.launch


rosbag record  /camera1/color/image_raw    /camera2/color/image_raw -o /data/static.bag

# fixing issue with cv_bridge 

root@ubuntu:/catkin_ws# export LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libgomp.so.1


rosrun kalibr kalibr_calibrate_cameras     --bag /data/static_2025-02-18-14-37-49.bag --target /data/april_6x6.yaml     --models pinhole-radtan pinhole-radtan     --topics /camera1/color/image_raw /camera2/color/image_raw --show-extraction  --bag-freq 10