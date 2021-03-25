/*
 @copyright Automotive Intelligence Lab, Konkuk University
 @author kimchuorok@gmail.com, pauljiwon96@gmail.com, yondoo20@gmail.com 
 @file visualization.cpp
 @Image Landmark and Semantic Point Cloud Map Matching Localization
 @version 1.0
 @date 2021-03-25
 */

#include "visualization.hpp"

LandMarkMatchingVisualizer::LandMarkMatchingVisualizer()
:i_trajectory_id_(0)
{
    int i_buffer_size = 10;
    rospub_float32_information_ = nh2.advertise<std_msgs::Float32>("float_info",i_buffer_size);
    rospub_trajectory_marker_ = nh2.advertise<visualization_msgs::Marker>("trajectory_marker",i_buffer_size);

    rossub_deadreckoning_ = nh2.subscribe("/particleEstimatedPose",i_buffer_size, &LandMarkMatchingVisualizer::CallbackDeadreckoningPose, this);
    rossub_novatel_enu_ = nh2.subscribe("/novatelENUPose",i_buffer_size, &LandMarkMatchingVisualizer::CallbackNovatelENUPose, this);
    rossub_ublox_enu_ = nh2.subscribe("/ubloxENUPose",i_buffer_size, &LandMarkMatchingVisualizer::CallbackUbloxENUPose, this);
}

LandMarkMatchingVisualizer::~LandMarkMatchingVisualizer()
{
}

void LandMarkMatchingVisualizer::Init()
{
    
}

void LandMarkMatchingVisualizer::CallbackDeadreckoningPose(const geometry_msgs::PoseStamped::ConstPtr& input)
{
    TrajectoryPublisher(input, "deadreckoning");
}

void LandMarkMatchingVisualizer::CallbackNovatelENUPose(const geometry_msgs::PoseStamped::ConstPtr& input)
{
    TrajectoryPublisher(input, "novatel");
}

void LandMarkMatchingVisualizer::CallbackUbloxENUPose(const geometry_msgs::PoseStamped::ConstPtr& input)
{
    TrajectoryPublisher(input, "ublox");
}

void LandMarkMatchingVisualizer::VehicleVelocityPublisher(float f_input, ros::Publisher rospub_pub)
{
    std_msgs::Float32 f32msg_input;
    f32msg_input.data = f_input;
    rospub_pub.publish(f32msg_input);
}

void LandMarkMatchingVisualizer::TrajectoryPublisher(const geometry_msgs::PoseStamped::ConstPtr& input, string str_namespace)
{
    visualization_msgs::Marker vmarker_trajectory_marker;
    

    vmarker_trajectory_marker.header = input->header;
    vmarker_trajectory_marker.ns = str_namespace;    

    if(str_namespace == "ublox"){
        vmarker_trajectory_marker.type = visualization_msgs::Marker::SPHERE;
        vmarker_trajectory_marker.action = visualization_msgs::Marker::ADD;
        vmarker_trajectory_marker.id = i_trajectory_id_++;    
        vmarker_trajectory_marker.scale.x = 0.5;
        vmarker_trajectory_marker.scale.y = 0.5;
        vmarker_trajectory_marker.scale.z = 0.3;
        vmarker_trajectory_marker.color.b = 1.0;
        vmarker_trajectory_marker.color.a = 1.0;
    }
    else if(str_namespace == "novatel"){
        vmarker_trajectory_marker.type = visualization_msgs::Marker::SPHERE;
        vmarker_trajectory_marker.action = visualization_msgs::Marker::ADD;
        vmarker_trajectory_marker.id = i_trajectory_id_++;    
        vmarker_trajectory_marker.scale.x = 0.5;
        vmarker_trajectory_marker.scale.y = 0.5;
        vmarker_trajectory_marker.scale.z = 0.3;
        vmarker_trajectory_marker.color.g = 1.0;
        vmarker_trajectory_marker.color.a = 1.0;
    }
    else if(str_namespace == "deadreckoning"){
        vmarker_trajectory_marker.type = visualization_msgs::Marker::SPHERE;
        vmarker_trajectory_marker.action = visualization_msgs::Marker::ADD;
        vmarker_trajectory_marker.id = i_trajectory_id_++;    
        vmarker_trajectory_marker.scale.x = 0.5;
        vmarker_trajectory_marker.scale.y = 0.5;
        vmarker_trajectory_marker.scale.z = 0.3;
        vmarker_trajectory_marker.color.b = 1.0;
        vmarker_trajectory_marker.color.r = 1.0;
        vmarker_trajectory_marker.color.a = 1.0;
    }else{
        vmarker_trajectory_marker.type = visualization_msgs::Marker::SPHERE;
        vmarker_trajectory_marker.action = visualization_msgs::Marker::ADD;
        vmarker_trajectory_marker.id = i_trajectory_id_++;    
        vmarker_trajectory_marker.scale.x = 0.3;
        vmarker_trajectory_marker.scale.y = 0.3;
        // vmarker_trajectory_marker.scale.z = 0.3;
        vmarker_trajectory_marker.color.r = 1.0;
        vmarker_trajectory_marker.color.a = 1.0;
    }
    
    vmarker_trajectory_marker.pose = input->pose;

    vmarker_trajectory_marker.points.push_back(input->pose.position);
    rospub_trajectory_marker_.publish(vmarker_trajectory_marker);
}

void LandMarkMatchingVisualizer::PoseWithCovariancePublisher(const geometry_msgs::PoseStamped::ConstPtr& input, string str_namespace, float f_std_dev)
{
    visualization_msgs::Marker vmarker_trajectory_marker;

    vmarker_trajectory_marker.header = input->header;
    vmarker_trajectory_marker.ns = str_namespace;
    
    double d_scale;
    if(f_std_dev*5<0.2)
    {
        d_scale = 0.2;
    }
    else if(f_std_dev*5>1.0)
    {
        d_scale = 1.0;
    }
    else
    {
        d_scale = f_std_dev*5;
    }

    if(str_namespace == "ublox"){
        vmarker_trajectory_marker.type = visualization_msgs::Marker::CYLINDER;
        vmarker_trajectory_marker.action = visualization_msgs::Marker::ADD;
        vmarker_trajectory_marker.id = i_trajectory_id_++;    
        vmarker_trajectory_marker.scale.x = 0.5;
        vmarker_trajectory_marker.scale.y = 0.5;
        vmarker_trajectory_marker.scale.z = 0.3;
        vmarker_trajectory_marker.color.b = 1.0;
        vmarker_trajectory_marker.color.a = 1.0;
    }
    else if(str_namespace == "novatel"){
        vmarker_trajectory_marker.type = visualization_msgs::Marker::CYLINDER;
        vmarker_trajectory_marker.action = visualization_msgs::Marker::ADD;
        vmarker_trajectory_marker.id = i_trajectory_id_++;    
        vmarker_trajectory_marker.scale.x = 0.5;
        vmarker_trajectory_marker.scale.y = 0.5;
        vmarker_trajectory_marker.scale.z = 0.3;
        vmarker_trajectory_marker.color.g = 1.0;
        vmarker_trajectory_marker.color.a = 1.0;
    }else{
        vmarker_trajectory_marker.type = visualization_msgs::Marker::CYLINDER;
        vmarker_trajectory_marker.action = visualization_msgs::Marker::ADD;
        vmarker_trajectory_marker.id = i_trajectory_id_++;    
        vmarker_trajectory_marker.scale.x = 0.3;
        vmarker_trajectory_marker.scale.y = 0.3;
        // vmarker_trajectory_marker.scale.z = 0.3;
        vmarker_trajectory_marker.color.r = 1.0;
        vmarker_trajectory_marker.color.a = 1.0;
    }
    
    vmarker_trajectory_marker.pose = input->pose;

    vmarker_trajectory_marker.points.push_back(input->pose.position);
    rospub_trajectory_marker_.publish(vmarker_trajectory_marker);
}

void LandMarkMatchingVisualizer::PoseTypePublisher(string str_type, int i_pose_type, ros::Publisher rospub_pub)
{
  string str_gps_status;
  std_msgs::String strmsg_string_input;

  if(str_type == "novatel")
  {
    switch(i_pose_type)
    {
      case 16:
        str_gps_status = "SINGLE";
      case 17:
        str_gps_status = "PSRDIFF";
      case 34:
        str_gps_status = "NARROW_FLOAT";
      case 49:
        str_gps_status = "WIDE_INT";
      case 50:
        str_gps_status = "NARROW_INT";
      case 53:
        str_gps_status = "INS_PSRSP";
      case 54:
        str_gps_status = "INS_PSRDIFF";
      case 55:
        str_gps_status = "INS_RTKFLOAT";
      case 56:
        str_gps_status = "INS_RTKFIXED";
    }

    str_gps_status = "Novatel pose type \n"+str_gps_status;
    strmsg_string_input.data = str_gps_status;    
  }
  else if(str_type == "best")
  {
    switch(i_pose_type)
    {
      case 16:
        str_gps_status = "SINGLE";
      case 17:
        str_gps_status = "PSRDIFF";
      case 34:
        str_gps_status = "NARROW_FLOAT";
      case 49:
        str_gps_status = "WIDE_INT";
      case 50:
        str_gps_status = "NARROW_INT";
      case 53:
        str_gps_status = "INS_PSRSP";
      case 54:
        str_gps_status = "INS_PSRDIFF";
      case 55:
        str_gps_status = "INS_RTKFLOAT";
      case 56:
        str_gps_status = "INS_RTKFIXED";
    }

    str_gps_status = "Best pose type \n"+str_gps_status;
    strmsg_string_input.data = str_gps_status;
  }
  else if(str_type == "nav")
  {
    switch(i_pose_type)
    {
      case -1:
        str_gps_status = "STATUS_NO_FIX";
      case 0:
        str_gps_status = "STATUS_FIX";
      case 1:
        str_gps_status = "STATUS_SBAS_FIX";
      case 2:
        str_gps_status = "STATUS_GBAS_FIX";
    }

    str_gps_status = "Ublox type \n"+str_gps_status;
    strmsg_string_input.data = str_gps_status;
  }
  else if(str_type == "ublox")
  {
    switch(i_pose_type)
    {
      case 0:
        str_gps_status = "FIX_TYPE_NO_FIX";
      case 1:
        str_gps_status = "FIX_TYPE_DEAD_RECKONING_ONLY";
      case 2:
        str_gps_status = "FIX_TYPE_2D";
      case 3:
        str_gps_status = "FIX_TYPE_3D";
      case 4:
        str_gps_status = "FIX_TYPE_GPS_DEAD_RECKONING_COMBINED";
      case 5:
        str_gps_status = "FIX_TYPE_TIME_ONLY";
    }

    str_gps_status = "GPS type \n"+str_gps_status;
    strmsg_string_input.data = str_gps_status;
  }

  rospub_pub.publish(strmsg_string_input);
}

// void LandMarkMatchingVisualizer::ParticlePublisher(const geometry_msgs::PoseArray::ConstPtr& input)
// {
//   visualization_msgs::MarkerArray particle_array;

//   for(auto const &)
// }


int main(int argc, char **argv)
{
  std::string str_node_name = "landmark_matching_visualizer";
	ros::init(argc, argv, str_node_name);

    LandMarkMatchingVisualizer LandMarkMatchingVisualizer;
    // LandMarkMatchingVisualizer.Initialization();

    int i_loop_freq = 10;

    ros::Rate rosrate_loop_rate(i_loop_freq);

    while(ros::ok())
    {
        ros::spinOnce();
        rosrate_loop_rate.sleep();
    }

    return 0;
}