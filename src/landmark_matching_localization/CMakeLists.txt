cmake_minimum_required(VERSION 3.0.2)
project(landmark_matching_localization)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)
set(ATTRIBUTE_FLAG "-Wignored-attributes")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${ATTRIBUTE_FLAG}")

find_package(OpenCV REQUIRED)
find_package(OpenCL REQUIRED)
find_package(PCL 1.3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  nav_msgs
  ublox_msgs
  novatel_oem7_msgs
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  visualization_msgs
  grid_map_ros
  grid_map_cv
  grid_map_msgs
  message_generation
)

find_package(Boost REQUIRED COMPONENTS system)

add_message_files(
  FILES
  Label.msg
  Labels.msg
  Point.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
    CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  # OpenCV
  ${OpenCV_INCLUDE_DIRS}
  # OpenCL
  ${OpenCL_INCLUDE_DIR}
  ${OpenCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(${PROJECT_NAME}
 src/landmark_matching_pf.cpp
)
add_executable(landmark_matching_visualizer
 src/visualization.cpp
)
add_executable(tf_broadcaster
 src/tf_broadcaster.cpp
)

add_executable(spc_map_processing
 src/spc_map_processing.cpp
)

target_include_directories(spc_map_processing PRIVATE nodes/spc_map_processing ${catkin_INCLUDE_DIRS})
target_link_libraries(spc_map_processing ${catkin_LIBRARIES}
	${PCL_COMMON_LIBRARIES}
  	${PCL_IO_LIBRARIES}
)
add_dependencies(spc_map_processing ${catkin_EXPORTED_TARGETS})

add_dependencies(${PROJECT_NAME} 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    # OpenCV
    ${OpenCV_LIBRARIES}
    # OpenCL
    ${OpenCL_LIBRARY}
    ${PCL_LIBRARIES}
)

add_dependencies(landmark_matching_visualizer 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(landmark_matching_visualizer
    ${catkin_LIBRARIES}
    # OpenCV
    ${OpenCV_LIBRARIES}
    # OpenCL
    ${OpenCL_LIBRARY}
    ${PCL_LIBRARIES}
)

add_dependencies(tf_broadcaster 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(tf_broadcaster
    ${catkin_LIBRARIES}
    # OpenCV
    ${OpenCV_LIBRARIES}
    # OpenCL
    ${OpenCL_LIBRARY}
    ${PCL_LIBRARIES}
)

