/*
 @copyright Automotive Intelligence Lab, Konkuk University
 @author kimchuorok@gmail.com, pauljiwon96@gmail.com, yondoo20@gmail.com 
 @file tf_broadcaster.cpp
 @Image Landmark and Semantic Point Cloud Map Matching Localization
 @version 1.0
 @date 2021-03-25
 */

#include "tf_broadcaster.hpp"

tfBroadcaster::tfBroadcaster()
:d_map_height_(15.0)
{
    int i_buffer_size = 1;
    rossub_novatel_inspvax_ = nh.subscribe("/novatel/oem7/inspvax", i_buffer_size, &tfBroadcaster::CallBackNovatelINSPVAX, this);
    rossub_ublox_hnrpvt_ = nh.subscribe("/ublox_msgs/hnrpvt", i_buffer_size, &tfBroadcaster::CallBackUbloxHNRPVT, this);
    rossub_estimated_pose_ = nh.subscribe("/particleEstimatedPose", i_buffer_size, &tfBroadcaster::CallBackEstimatedPose, this);
    rospub_novatel_pose_cheker_ = nh.advertise<geometry_msgs::PoseStamped>("/novatal/posestamp",10);
}

tfBroadcaster::~tfBroadcaster()
{

}

void tfBroadcaster::CallBackNovatelINSPVAX(const novatel_oem7_msgs::INSPVAX::ConstPtr &msg)
{
    static tf::TransformBroadcaster tfbroad_novatel;

    ros::Time rostime_pres_time = msg->header.stamp;

    tf::Quaternion tfquat_quaternion;
    tfquat_quaternion.setRPY(-(msg->roll) * M_PI / 180., -(msg->pitch) * M_PI / 180., (-1*msg->azimuth + 90.0) * M_PI / 180.);

    geometry_msgs::PoseStamped psstp_novatel_enu_pose = ConvertToMapFrame(msg->latitude, msg->longitude, msg->height);
    tf::Vector3 tfvec_offset = tf::Vector3(psstp_novatel_enu_pose.pose.position.x, psstp_novatel_enu_pose.pose.position.y, psstp_novatel_enu_pose.pose.position.z);
    tf::quaternionTFToMsg(tfquat_quaternion,psstp_novatel_enu_pose.pose.orientation);
    rospub_novatel_pose_cheker_.publish(psstp_novatel_enu_pose);

    tfbroad_novatel.sendTransform( 
        tf::StampedTransform(tf::Transform(tfquat_quaternion, tfvec_offset), rostime_pres_time, "map", "novatel"));
}

void tfBroadcaster::CallBackUbloxHNRPVT(const ublox_msgs::HnrPVT::ConstPtr &msg)
{
    static tf::TransformBroadcaster tfbroad_novatel;

    ros::Time rostime_pres_time = ros::Time::now();

    tf::Quaternion tfquat_quaternion;
    tfquat_quaternion.setRPY(0., 0., (-1 * msg->headVeh / 100000.0 + 90.0) * M_PI /180.);

    geometry_msgs::PoseStamped psstp_ublox_enu_pose = ConvertToMapFrame(msg->lat/10000000.0, msg->lon/10000000.0, msg->height/1000.0);
    tf::Vector3 tfvec_offset = tf::Vector3(psstp_ublox_enu_pose.pose.position.x, psstp_ublox_enu_pose.pose.position.y, psstp_ublox_enu_pose.pose.position.z);
    tf::quaternionTFToMsg(tfquat_quaternion,psstp_ublox_enu_pose.pose.orientation);

    tfbroad_novatel.sendTransform( 
        tf::StampedTransform(tf::Transform(tfquat_quaternion, tfvec_offset), rostime_pres_time, "map", "ublox"));
}

void tfBroadcaster::CallBackEstimatedPose(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    static tf::TransformBroadcaster tfbroad_estimated_pose;

    ros::Time rostime_pres_time = msg->header.stamp;

    double d_roll = 0.;
    double d_pitch = 0.;
    double d_yaw = 0.;

    tf::Quaternion tfquat_quaternion;
    quaternionMsgToTF(msg->pose.orientation,tfquat_quaternion);

    tf::Matrix3x3 tfmat_quat_matrix(tfquat_quaternion);
    tfmat_quat_matrix.getRPY(d_roll, d_pitch, d_yaw);

    tf::Vector3 tfvec_offset = tf::Vector3(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    d_map_height_ = msg->pose.position.z;

    tfbroad_estimated_pose.sendTransform( 
        tf::StampedTransform(tf::Transform(tfquat_quaternion, tfvec_offset), rostime_pres_time, "map", "estimated_pose"));

    tf::Vector3 tfvec_offset_velodyne = tf::Vector3(0,0,0.3);
    tf::Quaternion tfquat_quaternion_velodyne;
    tfquat_quaternion_velodyne.setRPY(0,0,0);

    tfbroad_estimated_pose.sendTransform( 
        tf::StampedTransform(tf::Transform(tfquat_quaternion_velodyne, tfvec_offset_velodyne), rostime_pres_time, "estimated_pose", "velodyne"));
}

geometry_msgs::PoseStamped tfBroadcaster::ConvertToMapFrame(float f_lat, float f_lon, float f_hgt)
{
    double d_kappa_lat = 0;
    double d_kappa_lon = 0;  

    double d_ref_latitude_deg = 37.3962732790;
    double d_ref_longigude_deg = 127.1066872418;

    f_hgt = d_map_height_;

    d_kappa_lat = FnKappaLat( d_ref_latitude_deg , f_hgt );
    d_kappa_lon = FnKappaLon( d_ref_latitude_deg , f_hgt );

    geometry_msgs::PoseStamped psstp_pose;

    psstp_pose.header.stamp = ros::Time::now();
    psstp_pose.header.frame_id = "map";

    psstp_pose.pose.position.x = (f_lon-d_ref_longigude_deg)/d_kappa_lon;
    psstp_pose.pose.position.y = (f_lat-d_ref_latitude_deg)/d_kappa_lat;
    psstp_pose.pose.position.z = f_hgt;

    return(psstp_pose);
}

double tfBroadcaster::FnKappaLat(double d_ref_latitude, double d_height)
{
	double d_kappa_lat = 0;
	double d_denominator = 0;
	double d_m = 0;

	d_denominator = sqrt(1 - D_GEOD_E2 * pow(sin(d_ref_latitude * D_DEG_2_RAD), 2));
	d_m = D_GEOD_A * (1 - D_GEOD_E2) / pow(d_denominator, 3);

	d_kappa_lat = 1 / (d_m + d_height) * D_RAD_2_DEG;

	return d_kappa_lat;
}
double tfBroadcaster::FnKappaLon(double d_ref_latitude, double d_height)
{
	double d_kappa_lon = 0;
	double d_denominator = 0;
	double d_n = 0;

	d_denominator = sqrt(1 - D_GEOD_E2 * pow(sin(d_ref_latitude * D_DEG_2_RAD), 2));
	d_n = D_GEOD_A / d_denominator;

	d_kappa_lon = 1 / ((d_n + d_height) * cos(d_ref_latitude * D_DEG_2_RAD)) * D_RAD_2_DEG;

	return d_kappa_lon;
}

int main(int argc, char **argv)
{
	std::string str_node_name = "tf_broadcaster";
	ros::init(argc, argv, str_node_name);
	ros::NodeHandle nh;

    tfBroadcaster tfBroadcaster;

    ros::spin();
	return 0;
}
