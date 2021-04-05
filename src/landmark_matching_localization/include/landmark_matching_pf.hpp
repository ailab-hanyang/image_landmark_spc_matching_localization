#ifndef __LANDMARK_MATCHING_PF_HPP__
#define __LANDMARK_MATCHING_PF_HPP__

#include <string>
#include <deque>
#include <iostream>
#include <cmath>
#include <vector>
#include <iomanip>
#include <map>
#include <random>
#include <Eigen/Dense>
#include <chrono>

// ROS header
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <ros/transport_hints.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Quaternion.h> // tf::quaternion
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <pcl/kdtree/kdtree_flann.h>

// Utility header
#include <pcl_conversions/pcl_conversions.h> // pcl_conversions::toPCL
#include <pcl/point_types.h>                 // pcl::PointXYZ
#include <pcl/PCLPointCloud2.h>              // pcl::PCLPointCloud2
#include <pcl/common/transforms.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/random_sample.h>
#include <pcl_ros/transforms.h>

#include"eigen3/Eigen/Dense"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/opencv.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/core/eigen.hpp"

// Message header
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "visualization_msgs/MarkerArray.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/PointCloud2.h"
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/Image.h>

#include <ublox_msgs/HnrPVT.h>
#include <visualization_msgs/Marker.h>
#include <novatel_oem7_msgs/BESTPOS.h>
#include <novatel_oem7_msgs/PositionOrVelocityType.h>
#include <novatel_oem7_msgs/INSPVAX.h>
#include <novatel_oem7_msgs/CORRIMU.h>

//Custom Message
#include "landmark_matching_localization/Labels.h"
#include "landmark_matching_localization/Label.h"
#include "landmark_matching_localization/Point.h"

#define _USE_MATH_DEFINES

class LandmarkMatchingPF
{
    struct GpsData{
        
        double sec;
        double nsec;
        double timestamp;
        double latitude;
        double longitude;
        double height;
        double yaw;
        double vel_ms;
        double quality;
        double latitude_std;
        double longitude_std;
        double height_std;
    };

    struct ImuData{
        
        double sec;
        double nsec;
        double timestamp; //msec
        double yaw_rate_rads;
        double acc_mss;
        double vel_ms;

    };

    ///////////////////////////////////////////////////////////////////
    // 0. Initialization
    public:
        explicit LandmarkMatchingPF();
        ~LandmarkMatchingPF();

        void Initialization();
       
    private:
        bool checker_b_is_first_IMU_step_;

    ///////////////////////////////////////////////////////////////////
    // 1. I/O
    private:
        ros::NodeHandle nh;
        ros::Subscriber rossub_novatel_inspvax_;
        ros::Subscriber rossub_novatel_corrimu_;
        ros::Subscriber rossub_ublox_hnrpvt_;
        ros::Subscriber rossub_velodyne_;
        ros::Subscriber rossub_map_height_;
        ros::Subscriber rossub_global_point_cloud_map_;
        ros::Subscriber rossub_global_semantic_point_cloud_map_;
        ros::Subscriber rossub_image_;
        ros::Subscriber rossub_local_point_cloud_map_;
        ros::Subscriber rossub_segmented_image_deeplab_;
        ros::Subscriber rossub_image_yolo_detection_label_;

        ros::Publisher rospub_novatel_enu_pose_;
        ros::Publisher rospub_ublox_enu_pose_;
        ros::Publisher rospub_transformed_point_cloud_;
        ros::Publisher rospub_cropped_point_cloud_;
        ros::Publisher rospub_estimated_pose_;
        ros::Publisher rospub_particle_pose_array_;
        ros::Publisher rospub_gps_quality_;
        ros::Publisher rospub_lowcost_gps_quality_;
        ros::Publisher rospub_error_pose_;
        ros::Publisher rospub_particle_marker_array_;
        ros::Publisher rospub_matching_score_;

        std::chrono::system_clock::time_point m_init_time;

        const double D_GEOD_A = 6378137.0;//SemiMajorAxis
        const double D_GEOD_E2 = 0.00669437999014; // FirstEccentricitySquard, e ^ 2
        const double D_RAD_2_DEG = 180 / M_PI;
        const double D_DEG_2_RAD = M_PI / 180;

        ImuData imu_imu_data_;
        GpsData gps_gps_data_;
        GpsData gps_ublox_data_;
        GpsData gps_init_gps_data;
        GpsData gps_init_ublox_data;

        geometry_msgs::PoseStamped psstp_init_gnss_enu_pose_;
        geometry_msgs::PoseStamped psstp_init_ublox_enu_pose_;
        geometry_msgs::PoseStamped psstp_gnss_enu_pose_;

        std::string strmsg_result_path_;
        std::string m_str_fileName;

        double array_ego_vehicle_pose_[3];
        double d_map_height_;
        double d_map_roll_;
        double d_map_pitch_;

        double param_d_input_vel_sigma_ms_;
        double param_d_input_yaw_rate_sigma_degs_;
        double param_d_input_yaw_rate_sigma_rads_;
        double param_d_input_acc_sigma_mss_;
        double param_d_east_sigma_m_;
        double param_d_north_sigma_m_;
        double param_d_heading_sigma_deg_;
        double param_d_heading_sigma_rad_;
        double param_d_resampling_coefficent_;
        double param_d_gnss_radius_boundary_m_;
        double param_d_gnss_position_quality_threshold_m_;
        double param_d_gnss_fault_position_threshold_m_;
        double param_d_gnss_fault_heading_threshold_deg_;

        bool param_b_refernce_gps_novatel_;
        bool param_b_semantic_map_use_;

        bool checker_b_imu_data_exist_;
        bool checker_b_gps_data_exist_;
        bool checker_b_ublox_data_exist_;
        bool checker_b_gnss_data_init_;
        bool checker_b_reference_pose_data_init_;
        bool checker_b_initialization_complete_;
        bool checker_b_ublox_data_init_;
        bool checker_b_map_data_init_;

        int i_num_of_state_;
        int param_i_num_particle;

    private:
        std::default_random_engine random_generator_;

    // Particles
    private:
        Eigen::MatrixXd* egmat_timestamped_particle_state_;
        Eigen::MatrixXd* egmat_timestamped_particle_state_resampling_;
        Eigen::MatrixXd* egmat_particle_weight_;
        Eigen::MatrixXd* egmat_estimated_state_;
        Eigen::MatrixXd* egmat_estimated_sigma_;
        Eigen::MatrixXd* egmat_gps_particle_likelihood_;
        Eigen::MatrixXd* egmat_map_matching_particle_likelihood_;

    // Switching Mode
    private:
        bool param_b_localization_type_point_cloud_map_matching_;
        bool param_b_localization_type_spc_image_matching_;
        bool param_b_localization_type_gps_;

    // Point Cloud Map Matching Variables
    private:
        int param_i_num_map_matching_random_sample_point_;
        double param_d_map_noise_measure_noise_sigma_m_;
        double param_d_map_sigma_;

        pcl::KdTreeFLANN<pcl::PointXYZRGB>::Ptr kdtree_global_map_;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcptr_input_point_cloud_;

    // Image & SPC Matching Variables
    private:
        bool param_b_polygon_view_on_;
        bool param_b_segmented_view_on_;

        int i_frame_count;
        int param_i_visualize_particle_index_;
        
        bool checker_b_image_data_exist_;
        bool checker_b_image_contour_data_exist_;
        bool checker_b_segmented_image_data_exist_;

        int param_i_num_sampling_building_;
        int param_i_num_sampling_fence_;
        int param_i_num_sampling_lane_marking_;
        int param_i_num_sampling_trunk_;
        int param_i_num_sampling_pole_;
        int param_i_num_sampling_traffic_sign_;
        int param_i_num_sampling_traffic_light_;
        int param_i_num_sampling_tunnel_fan_;
        int param_i_num_sampling_tunnel_light_;
        int param_i_num_sampling_tunnel_hydrant_;

        double param_d_lane_min_x_roi_m_;
        double param_d_lane_max_x_roi_m_;
        double param_d_lane_min_y_roi_m_;
        double param_d_lane_max_y_roi_m_;

        double param_d_building_min_x_roi_m_;
        double param_d_building_max_x_roi_m_;
        double param_d_building_min_y_roi_m_;
        double param_d_building_max_y_roi_m_;

        double param_d_fence_min_x_roi_m_;
        double param_d_fence_max_x_roi_m_;
        double param_d_fence_min_y_roi_m_;
        double param_d_fence_max_y_roi_m_;

        double param_d_tunnel_min_x_roi_m_;
        double param_d_tunnel_max_x_roi_m_;
        double param_d_tunnel_min_y_roi_m_;
        double param_d_tunnel_max_y_roi_m_;

        double param_d_traffic_min_x_roi_m_;
        double param_d_traffic_max_x_roi_m_;
        double param_d_traffic_min_y_roi_m_;
        double param_d_traffic_max_y_roi_m_;

        double param_d_pole_min_x_roi_m_;
        double param_d_pole_max_x_roi_m_;
        double param_d_pole_min_y_roi_m_;
        double param_d_pole_max_y_roi_m_;

        double param_d_pole_likelihood_thresholde_;
        double param_d_traffic_sign_likelihood_thresholde_;
        double param_d_traffic_light_likelihood_threshold_;
        double param_d_tunnel_fan_likelihood_threshold_;
        double param_d_tunnel_light_likelihood_threshold_;
        double param_d_tunnel_hydrant_likelihood_threshold_;
        double param_d_building_likelihood_threshold_;
        double param_d_lane_marking_likelihood_threshold_;
        double param_d_fence_likelihood_threshold_;

        double param_d_extrinsic_calibration_roll_deg_;
        double param_d_extrinsic_calibration_pitch_deg_;
        double param_d_extrinsic_calibration_yaw_deg_;
        double param_d_extrinsic_calibration_x_m_;
        double param_d_extrinsic_calibration_y_m_;
        double param_d_extrinsic_calibration_z_m_;

        double param_d_imu_height_m_;

        double calibration_array_camera_intrinsic_matrix_[9]         = {2430.            , 0.               , 990.0000000000000, 
                                                                        0.               , 2430.            , 590.0000000000000, 
                                                                        0.               , 0.               , 1.                };
        double calibration_array_camera_distortion_matrix_[5]      = {-0.119197654412551, 0.133075890338234, 0., 0., 0.};

        double calibration_array_zero_rotation_matrix_[9]   = {1., 0., 0.,
                                      0., 1., 0., 
                                      0., 0., 1.};
        double calibration_array_zero_translation_vector_[3]         = {0.,0.,0.};

        int i_pointcloud_2d_projection_range_m_ = 50;
        double d_max_range_projected_point_cloud_m_;
        double d_min_range_projected_point_cloud_m_;
        double d_input_image_size_;

        cv::Vec3b cvvec_building_semantic_rgb_ = {0,0,128};
        cv::Vec3b cvvec_lane_semantic_rgb_ = {0,128,0};
        cv::Vec3b cvvec_fence_semantic_rgb_ = {128,0,0};

        // Image Polygon & Bounding box
        std::vector<std::vector<cv::Point>> vec_image_contour_point_pole_; // pole
        std::vector<std::vector<cv::Point>> vec_image_contour_point_traffic_sign_; // traffic sign
        std::vector<std::vector<cv::Point>> vec_image_contour_point_traffic_light_; // traffic light
        std::vector<std::vector<cv::Point>> vec_image_contour_point_tunnel_fan_; // tunnel fan
        std::vector<std::vector<cv::Point>> vec_image_contour_point_tunnel_light_; // tunnel light
        std::vector<std::vector<cv::Point>> vec_image_contour_point_tunnel_hydrant; // tunnel hydrant

        std::vector<std::vector<cv::Point2d>> vec_particle_based_projected_points_building_;
        std::vector<std::vector<cv::Point2d>> vec_particle_based_projected_points_lane_marking_;
        std::vector<std::vector<cv::Point2d>> vec_particle_based_projected_points_fence_;
        std::vector<std::vector<cv::Point2d>> vec_particle_based_projected_points_trunk_;
        std::vector<std::vector<cv::Point2d>> vec_particle_based_projected_points_pole_;
        std::vector<std::vector<cv::Point2d>> vec_particle_based_projected_points_traffic_sign_;
        std::vector<std::vector<cv::Point2d>> vec_particle_based_projected_points_traffic_light_;
        std::vector<std::vector<cv::Point2d>> vec_particle_based_projected_points_tunnel_fan_;
        std::vector<std::vector<cv::Point2d>> vec_particle_based_projected_points_tunnel_light_;
        std::vector<std::vector<cv::Point2d>> vec_particle_based_projected_points_tunnel_hydrant_;

        std::vector<cv::Point2d> vec_ref_pose_based_projected_point_building_;
        std::vector<cv::Point2d> vec_ref_pose_based_projected_point_fence_;
        std::vector<cv::Point2d> vec_ref_pose_based_projected_point_lane_marking_;
        std::vector<cv::Point2d> vec_ref_pose_based_projected_point_trunk_;
        std::vector<cv::Point2d> vec_ref_pose_based_projected_point_pole_;
        std::vector<cv::Point2d> vec_ref_pose_based_projected_point_traffic_sign_;
        std::vector<cv::Point2d> vec_ref_pose_based_projected_point_traffic_light_;
        std::vector<cv::Point2d> vec_ref_pose_based_projected_point_tunnel_fan_;
        std::vector<cv::Point2d> vec_ref_pose_based_projected_point_tunnel_light_;
        std::vector<cv::Point2d> vec_ref_pose_based_projected_point_tunnel_hydrant_;

        std::vector<cv::Point2d> vec_inside_contour_points_;
        std::vector<cv::Point2d> vec_outside_contour_points_;

        std::vector<cv::Point2d> vec_inside_segmented_points_;
        std::vector<cv::Point2d> vec_near_segmented_points_;
        std::vector<cv::Point2d> vec_outside_segmented_points_;

        cv_bridge::CvImagePtr cvptr_input_image_;
        sensor_msgs::ImageConstPtr imgmsg_input_image_;
        
        cv::Mat cvmat_segmented_image_;
        Eigen::Matrix4d egmat_lidar_vehicle_transform_;
        std::vector<Eigen::Matrix4d> vec_ref_par_transform_;

        Eigen::MatrixXd* egmat_building_particle_likelihood_;
        Eigen::MatrixXd* egmat_lane_marking_particle_likelihood_;
        Eigen::MatrixXd* egmat_fence_particle_likelihood_;
        Eigen::MatrixXd* egmat_pole_particle_likelihood_;
        Eigen::MatrixXd* egmat_traffic_sign_particle_likelihood_;
        Eigen::MatrixXd* egmat_traffic_light_particle_likelihood_;
        Eigen::MatrixXd* egmat_tunnel_fan_particle_likeilhood_;
        Eigen::MatrixXd* egmat_tunnel_light_particle_likelihood_;
        Eigen::MatrixXd* egmat_tunnel_hydrant_particle_likelihood_;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcptr_building_sampled_point_cloud_;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcptr_fence_sampled_point_cloud_;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcptr_lane_marking_sampled_point_cloud_;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcptr_trunk_sampled_point_cloud_;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcptr_pole_sampled_point_cloud_;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcptr_traffic_sign_sampled_point_cloud_;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcptr_traffic_light_sampled_point_cloud_;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcptr_tunnel_fan_sampled_point_cloud_;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcptr_tunnel_light_sampled_point_cloud_;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcptr_tunnel_hydrant_sampled_point_cloud_;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcptr_input_local_point_cloud_map_;

    public:
        void Run();
        void GetParameter();

    // CALLBACK FUNCTIONS
    private:
        void CallBackGlobalSemanticPointCloudMap(const sensor_msgs::PointCloud2::ConstPtr &msg);
        void CallbackSegmentedImage(const sensor_msgs::Image::ConstPtr& msg);
        void CallbackImageDetectionLabel(const landmark_matching_localization::Labels::ConstPtr &msg);
        void CallBackGlobalPointCloudMap(const sensor_msgs::PointCloud2::ConstPtr &msg);
        void CallBackLocalPointCloudMap(const sensor_msgs::PointCloud2::ConstPtr &msg);
        void CallBackVelodyne(const sensor_msgs::PointCloud2::ConstPtr &msg);
        void CallBackMapHeight(const geometry_msgs::Pose::ConstPtr &msg);
        void CallbackImage(const sensor_msgs::ImageConstPtr& input);
        void CallBackNovatelINSPVAX(const novatel_oem7_msgs::INSPVAX::ConstPtr &msg);
        void CallBackNovatelCORRIMU(const novatel_oem7_msgs::CORRIMU::ConstPtr &msg);
        void CallBackUbloxHNRPVT(const ublox_msgs::HnrPVT::ConstPtr &msg);

    private:
    // 0. Fault Detection & Particle Initialization(Reset)
        bool PFParticleInit(double array_state[]);
        bool PFFaultDetection(void);

    // 1. Prediction
        bool PFPrediction(void);
        
    // 2. Representative Pose Extraction
        bool PFRepresentativePoseExtraction(void);   

    // 3. Resampling
        bool PFResampling(void);
    
    // 4. Measurement Update
        void WeightUpdate(Eigen::MatrixXd* egmat_input_weight_mat);
        bool NormalizationWeight(Eigen::MatrixXd* egmat_input_weight_mat);
        
    // 4-1. GPS based Measurement Update
        void GPSMeasurementUpdate(void);

    // 4-2. Point Cloud Map Matching based Measurement Update
        void MapMatchingMeasurementUpdate(void);

    // 4-3. Image & SPC Matching based Measurement Update
        void ImageSPCMeasurementUpdate(void);
        double ImageContourSPCMatching(std::vector<std::vector<cv::Point>> vec_input_contours, std::vector<cv::Point2d> vec_input_points);
        double SegmentedImageSPCMatching(std::vector<cv::Point2d> vec_input_points, cv::Vec3b cvvec_ref_bgr, int i_pixel_bias);
    
    // 4-3-1. Util Function
        void PointCloud2DProjection(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputPointCloud, Eigen::Matrix4d egmat_input_mat, std::vector<cv::Point2d>& vec_output_2d_projected_point);
        void ParticlePointsTransformation(void);
        int CountNearbyIntersectionPoint(cv::Point2d cv_point, cv::Vec3b cvvec_ref_bgr, int d_bias);
        bool CheckNearbyIntersectionPointExist(cv::Point2d cv_point, cv::Vec3b cvvec_ref_bgr, int d_bias);
        int CountPixels(const cv::Mat &image, cv::Vec3b cvvec_color);
        double ContourAreaCalc(std::vector<cv::Point> cv_point);     
        double CheckPointsInsideContour(std::vector<cv::Point> cv_point, cv::Point2d cvp_check_point);
        double EigenMatrixSum(Eigen::MatrixXd* egmat_input_matrix);
        
    // 4-3-2. Visualization Functions
        void ReferenceImagePointViewer(std::vector<cv::Point2d> vec_input_points);
        void ReferenceSegmentedImagePointAndCheckInsideViewer();
        void ReferenceImagePointAndCheckInsideViewer();
        void ParticleImagePointViewer(std::vector<cv::Point2d> vec_input_points, int i_particle_num);
        double ImageSPCMatchingVisualization(std::vector<std::vector<cv::Point>> vec_input_contours, std::vector<cv::Point2d> vec_input_points);
        double SegmentedImageSPCMatchingVisualization(std::vector<cv::Point2d> vec_input_points, cv::Vec3b cvvec_ref_bgr, int i_pixel_bias);

    // ETC
        geometry_msgs::PoseStamped ConvertToMapFrame(float f_lat, float f_lon, float f_hgt);
        double GaussianRandomGenerator(double d_mean, double d_sigma);
        void SaveResultText(Eigen::MatrixXd* input, std::string mathcingType);
        void PublishParticlePoseArray(void);      
};
#endif