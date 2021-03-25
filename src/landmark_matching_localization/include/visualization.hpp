#ifndef __LANDMARK_MATCHING_VISUALIZER_HPP__
#define __LANDMARK_MATCHING_VISUALIZER_HPP__

#include <string>
#include <deque>
#include <iostream>
#include <cmath>

// ROS header
#include <ros/ros.h>
#include <ros/transport_hints.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Quaternion.h> // tf::quaternion
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

// Utility header
#include <pcl_conversions/pcl_conversions.h> // pcl_conversions::toPCL
#include <pcl/point_types.h>                 // pcl::PointXYZ
#include <pcl/PCLPointCloud2.h>              // pcl::PCLPointCloud2
#include <pcl/common/transforms.h>

// Message header
#include "geometry_msgs/PoseArray.h"
#include "visualization_msgs/MarkerArray.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/PointCloud2.h"
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include "nav_msgs/Odometry.h"

// Namespace
using namespace ros;
using namespace tf;
using namespace std;

typedef pcl::PointCloud<pcl::PointXYZI> PointCloudXYZI;

class LandMarkMatchingVisualizer 
{
public:
    explicit LandMarkMatchingVisualizer();
    ~LandMarkMatchingVisualizer();

    void Init();
    void InitPublisher();
    // void StringPublisher(string input);
    void VehicleVelocityPublisher(float input, ros::Publisher pub);
    void TrajectoryPublisher(const geometry_msgs::PoseStamped::ConstPtr& input, string ns);
    void PoseWithCovariancePublisher(const geometry_msgs::PoseStamped::ConstPtr& input, string ns, float std_dev);
    void ParticlePublisher(const geometry_msgs::PoseArray::ConstPtr& input);
    void PoseTypePublisher(string type, int pos_type, ros::Publisher pub);

    void CallbackDeadreckoningPose(const geometry_msgs::PoseStamped::ConstPtr& input);
    void CallbackNovatelENUPose(const geometry_msgs::PoseStamped::ConstPtr& input);
    void CallbackUbloxENUPose(const geometry_msgs::PoseStamped::ConstPtr& input);
    
// Ros 
private:    
    ros::NodeHandle nh2;
    ros::Publisher rospub_float32_information_;
    ros::Publisher rospub_trajectory_marker_;
    ros::Publisher rospub_odometry_;
    
    ros::Subscriber rossub_deadreckoning_;
    ros::Subscriber rossub_novatel_enu_;
    ros::Subscriber rossub_ublox_enu_;
    
// Call back function
private:

// Launch Parameter
private:

// Sensor input
private:
    int i_trajectory_id_;
};

#endif // __POINT_CLOUD_MAP_MATCHING_PF_HPP__