#ifndef __SPCMAPPROCESSING_HPP__
#define __SPCMAPPROCESSING_HPP__
// pcl 
#include <iostream>
#include <cstdlib>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/PCLPointCloud2.h>              // pcl::PCLPointCloud2
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_ros/transforms.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Quaternion.h> // tf::quaternion
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <pcl/kdtree/kdtree_flann.h>


#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

// ROS header
#include <ros/ros.h>
#include <tf/tf.h>
#include <std_msgs/Float32.h>

// Message header
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>

// Defines
#define FRAMEID "map"

using namespace std;

class SPCMapProcessing {
public:
    explicit SPCMapProcessing();
    ~SPCMapProcessing();

public:
    void PoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void ROIMapPublisher(void);
   
    template<typename T>
    bool MapReaderFromPCDToPCLXYZ(pcl::PointCloud<T> &input);
    bool MapReaderFromPCDToPointcloud2(pcl::PCLPointCloud2 &input);
    
    template<typename T>
    bool PCL2pointcloud2(pcl::PointCloud<T> &input, sensor_msgs::PointCloud2 &pc2_point_cloud_rgb);
    bool BuildKdtree(void);
    
    void GetPointCloudROI(double fX_m, double fY_m, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &output);
    double GetLowestHeight(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input);
    double GetAverageHeight(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input);
    geometry_msgs::Pose GetMapBasedRollPitch(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input);

    void PointCloudPublisher();
private:
    // Node Init
    ros::NodeHandle m_rosNodeHandler;

    // Publisher
    ros::Publisher rospub_map_rgb_;
    ros::Publisher rospub_map_intensity_;
    ros::Publisher rospub_map_pose_;
    ros::Publisher rospub_local_map_;
    ros::Publisher rospub_roi_map_;
    ros::Publisher rospub_roi_height_map_;
    ros::Publisher rospub_roi_transformed_map_;

    // Subscriber
    ros::Subscriber rossub_ego_pose_;
    
    // Member variable
    string param_str_pcd_file_path_;
    double param_d_roi_x_boundary_length_m_;
    double param_d_roi_y_boundary_length_m_;

    double d_height_m_;

    // Mode Flag
    double param_DEBUG_MODE_;
    double param_height_detection_type_minimum_;

    // Checker
    bool checker_b_is_init_height_;
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcptr_point_cloud_rgb_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcptr_point_cloud_intensity_;
    sensor_msgs::PointCloud2 pc2_point_cloud_rgb_ ;
    sensor_msgs::PointCloud2 pc2_point_cloud_intensity_ ;
    pcl::PCLPointCloud2 pclpc2_point_cloud_filtered_;

    pcl::KdTreeFLANN<pcl::PointXYZRGB>::Ptr kdtree_map_;
    pcl::KdTreeFLANN<pcl::PointXY>::Ptr kdtree_z_removed_map_;

    geometry_msgs::PoseStamped psstp_curr_enu_pose_;
};
#endif