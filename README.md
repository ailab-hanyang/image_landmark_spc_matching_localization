# Image Landmark and Semantic Point Cloud Map Matching based Localization

---------------------------------------------------------------------

### Package Install
```
git clone https://github.com/soyeongkim/image_landmark_spc_matching_localization.git
cd image_landmark_spc_matching_localization
catkin_make --cmake-args \
            -DCMAKE_BUILD_TYPE=Release \
            -DPYTHON_EXECUTABLE=/usr/bin/python3 \
            -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m \
            -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so
source devel/setup.bash
```
---------------------------------------------------------------------
### Dependencies

This software is build on ROS(Robotic Operating System).
Additionaly, it also requires the following libraries.
 - OpenCV
 - OpenCL
 - cuDNN
 - CUDA (recommand version 10.1)
 - Python (recommand version 3.5, 3.7)
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

You can download semantic point cloud map example file at https://konkukackr-my.sharepoint.com/:u:/g/personal/ailab_konkuk_ac_kr/EWD-5vFFooNNjlrsSBbDGqgB6qHSD786wffZwWa2C8zB_A?e=0vTyMh. 

Put this file at src/landmark_matching_localization/map.

### Download Yolo Weights Example

You can download weights example file at https://konkukackr-my.sharepoint.com/:u:/g/personal/ailab_konkuk_ac_kr/EVMou4zEG-dOt5KNdtkBj3MBBucZqrhVZxit-auWfwoGmQ?e=4R8Nsm .

Put this file at src/darknet_ros/darknet_ros/yolo_network_config/weights.

### Download ROSBAG Example

You can download rosbag example file at

#### Tunnel Version : https://konkukackr-my.sharepoint.com/:u:/g/personal/ailab_konkuk_ac_kr/EcRktnkEg2NKtUdIOgrobsgBDoFPSP3hMAUaffJ6Vo_0Mg?e=LVdhhQ
#### Downtown Version : https://konkukackr-my.sharepoint.com/:u:/g/personal/ailab_konkuk_ac_kr/EWBhJB657SVEoqeyHpNLsZwBb7roT505lA1wo2m6qDQAwQ?e=3JV7Cf

```
rosbag play 15_ccw_in_d_c_2.bag
rosbag play 15_ccw_in_d_c_4.bag
```
