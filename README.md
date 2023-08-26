# Image Landmark and Semantic Point Cloud Map Matching based Localization

<img src="./holistic_matching/66_spc_holistic_matching.gif" width="500">

---------------------------------------------------------------------

### Dependencies

This software is build on ROS(Robotic Operating System).
Additionaly, it also requires the following libraries.
 - OpenCV (3.2)
 - cuDNN
 - CUDA (recommand version 10.1)
 - Python (recommand version 3.5, 3.7)

```
sudo apt-get update
sudo apt-get install ros-melodic-ublox
sudo apt-get install ros-melodic-novatel-oem7-driver
sudo apt-get install ros-melodic-grid-map
pip3 install empy
pip3 install catkin_pkg
```
---------------------------------------------------------------------
### Package Install
```
git clone https://github.com/ailab-konkuk/image_landmark_spc_matching_localization.git
cd image_landmark_spc_matching_localization
catkin_make --cmake-args \
            -DCMAKE_BUILD_TYPE=Release \
            -DPYTHON_EXECUTABLE=/usr/bin/python3 \
            -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m \
            -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so
source devel/setup.bash
```
#### * If you want to use only point cloud map matching localization, build only 'landmark_matching_localization'
```
catkin_make -DCMAKE_BUILD_TYPE=Release -DCATKIN_BLACKLIST_PACKAGES='darknet_ros;deeplab_ros'
```

---------------------------------------------------------------------
### Run the demo
#### 1. GPS based Localization
```
roslaunch landmark_matching_localization gps_localization.launch 
```
#### 2. Point Cloud Map Matching Localization
```
roslaunch landmark_matching_localization map_matching_localization.launch 
```
#### 3. Image Landmark and Semantic Point Cloud Map Matching Localization
For using image landmark and semantic point cloud map matching localization, you have to run 3 launch files.
(localization, image segmentation AI, image detection AI)

1. Image Landmark and Semantic Point Cloud Map Matching Localization
```
source devel/setup.bash
roslaunch landmark_matching_localization landmark_matching_localizaion.launch 
```
2. Image Segmentation AI
```
source devel/setup.bash
rosrun deeplab_ros inference_deeplab.py
```
3. Image Detection AI
```
source devel/setup.bash
roslaunch darknet_ros darknet_ros.launch
```

### Download Semantic Point Cloud Example

You can download semantic point cloud map example file at http://gofile.me/5s2Yk/uN3afXdtD

Put this file at src/landmark_matching_localization/map.

### Download Yolo Weights Example

You can download weights example file at http://gofile.me/5s2Yk/orOrQn5ND

Put this file at src/darknet_ros/darknet_ros/yolo_network_config/weights.

### Download ROSBAG Example

You can download rosbag example file at

#### Tunnel Version : http://gofile.me/5s2Yk/pQCSIBYwK
#### Downtown Version : http://gofile.me/5s2Yk/5S2GyFTkm

```
rosbag play 15_ccw_in_d_c_2.bag
rosbag play 15_ccw_in_d_c_4.bag
```
