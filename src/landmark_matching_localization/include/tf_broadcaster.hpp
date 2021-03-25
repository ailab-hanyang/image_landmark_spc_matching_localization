#ifndef __TFBROADCASTER_HPP__
#define __TFBROADCASTER_HPP__

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

#include <novatel_oem7_msgs/INSPVAX.h>
#include <ublox_msgs/HnrPVT.h>
#include "geometry_msgs/PoseStamped.h"

// Namespace
using namespace ros;
using namespace tf;
using namespace std;

class tfBroadcaster
{
    public:
        explicit tfBroadcaster();
        ~tfBroadcaster();

        double d_map_height_;
        const double D_GEOD_A = 6378137.0;//SemiMajorAxis
        const double D_GEOD_E2 = 0.00669437999014; // FirstEccentricitySquard, e ^ 2
        const double D_RAD_2_DEG = 180 / M_PI;
        const double D_DEG_2_RAD = M_PI / 180;
    
    private:
        ros::NodeHandle nh;
        ros::Subscriber rossub_novatel_inspvax_;
        ros::Subscriber rossub_estimated_pose_;
        ros::Subscriber rossub_ublox_hnrpvt_;

        ros::Publisher rospub_novatel_pose_cheker_;

        void CallBackNovatelINSPVAX(const novatel_oem7_msgs::INSPVAX::ConstPtr &msg);
        void CallBackUbloxHNRPVT(const ublox_msgs::HnrPVT::ConstPtr &msg);
        void CallBackEstimatedPose(const geometry_msgs::PoseStamped::ConstPtr &msg);
        void CallBackMapHeight(const geometry_msgs::Pose::ConstPtr &msg);

        geometry_msgs::PoseStamped ConvertToMapFrame(float lat, float lon, float hgt);
        double FnKappaLat(double dRef_Latitude, double dHeight);
        double FnKappaLon(double dRef_Latitude, double dHeight);
};

#endif