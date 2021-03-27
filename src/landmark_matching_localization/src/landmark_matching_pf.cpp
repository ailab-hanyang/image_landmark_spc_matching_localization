/*
 * @copyright Automotive Intelligence Lab, Konkuk University
 * @author kimchuorok@gmail.com, pauljiwon96@gmail.com, yondoo20@gmail.com 
 * @file landmark_matching.cpp
 * @brief Image Landmark and Semantic Point Cloud Map Matching Localization
 * @version 1.0
 * @date 2021-03-25
 */

#include "landmark_matching_pf.hpp"

LandmarkMatchingPF::LandmarkMatchingPF()
:checker_b_is_first_IMU_step_(false)  , checker_b_imu_data_exist_(false)            , checker_b_gps_data_exist_(false)          , checker_b_ublox_data_exist_(false)   , param_b_localization_type_spc_image_matching_(false),
checker_b_gnss_data_init_(false)      , checker_b_reference_pose_data_init_(false)  , checker_b_initialization_complete_(false) , checker_b_map_data_init_(false)      , d_map_height_(7.0),
param_i_num_particle(10)              , i_num_of_state_(3)                          , param_d_heading_sigma_deg_(0.0)           , param_b_refernce_gps_novatel_(true)  ,
param_d_input_vel_sigma_ms_(0.0)      , param_d_input_yaw_rate_sigma_degs_(0.0)     , param_d_input_yaw_rate_sigma_rads_(0.0)   , param_d_input_acc_sigma_mss_(0.0)    , 
param_d_east_sigma_m_(0.0)            , param_d_north_sigma_m_(0.0)                 , param_d_heading_sigma_rad_(0.0)           , param_b_localization_type_point_cloud_map_matching_(false) ,
param_d_gnss_radius_boundary_m_(2.0)  , param_d_resampling_coefficent_(1.0)         , param_i_num_map_matching_random_sample_point_(1000)    , param_d_map_noise_measure_noise_sigma_m_(0.2) ,
param_d_map_sigma_(10.0)              , checker_b_image_data_exist_(false)          , checker_b_image_contour_data_exist_(false)             , checker_b_segmented_image_data_exist_(false)  ,i_frame_count(0),
d_map_roll_(0.)                       , d_map_pitch_(0.)         ,
pcptr_input_local_point_cloud_map_(new pcl::PointCloud<pcl::PointXYZRGB>) ,
pcptr_building_sampled_point_cloud_(new pcl::PointCloud<pcl::PointXYZRGB>)     , pcptr_fence_sampled_point_cloud_(new pcl::PointCloud<pcl::PointXYZRGB>)         , 
pcptr_lane_marking_sampled_point_cloud_(new pcl::PointCloud<pcl::PointXYZRGB>) , pcptr_trunk_sampled_point_cloud_(new pcl::PointCloud<pcl::PointXYZRGB>)         , 
pcptr_pole_sampled_point_cloud_(new pcl::PointCloud<pcl::PointXYZRGB>)         , pcptr_traffic_sign_sampled_point_cloud_(new pcl::PointCloud<pcl::PointXYZRGB>)  , 
pcptr_traffic_light_sampled_point_cloud_(new pcl::PointCloud<pcl::PointXYZRGB>), pcptr_tunnel_fan_sampled_point_cloud_(new pcl::PointCloud<pcl::PointXYZRGB>)    , 
pcptr_tunnel_light_sampled_point_cloud_(new pcl::PointCloud<pcl::PointXYZRGB>) , pcptr_tunnel_hydrant_sampled_point_cloud_(new pcl::PointCloud<pcl::PointXYZRGB>)
{
    GetParameter();
    int i_buffer_size = 1;
    int i_map_msg_buffer_size = 1000;

    egmat_lidar_vehicle_transform_ << 1., 0., 0., 0., 
                               0., 1., 0., 0., 
                               0., 0., 1., 0.,
                               0., 0., 0., 1.;

    rossub_novatel_inspvax_           = nh.subscribe("/novatel/oem7/inspvax", i_buffer_size, &LandmarkMatchingPF::CallBackNovatelINSPVAX, this);
    rossub_novatel_corrimu_           = nh.subscribe("/novatel/oem7/corrimu", i_buffer_size, &LandmarkMatchingPF::CallBackNovatelCORRIMU, this);
    rossub_ublox_hnrpvt_              = nh.subscribe("/ublox_msgs/hnrpvt", i_buffer_size, &LandmarkMatchingPF::CallBackUbloxHNRPVT, this);
    rossub_velodyne_                  = nh.subscribe("/velodyne_points", i_buffer_size, &LandmarkMatchingPF::CallBackVelodyne, this);
    rossub_map_height_                = nh.subscribe("/map_height", i_buffer_size, &LandmarkMatchingPF::CallBackMapHeight, this);
    rossub_image_                     = nh.subscribe("cam2/pylon_camera_node/image_raw", 1, &LandmarkMatchingPF::CallbackImage, this);


    rossub_segmented_image_deeplab_           = nh.subscribe("/segmentation_image", i_buffer_size, &LandmarkMatchingPF::CallbackSegmentedImage, this);
    rossub_image_yolo_detection_label_        = nh.subscribe("/darknet_ros/inference_yolo_output", i_buffer_size, &LandmarkMatchingPF::CallbackImageDetectionLabel, this);
    rossub_global_point_cloud_map_            = nh.subscribe("/global_map_RGB", i_map_msg_buffer_size, &LandmarkMatchingPF::CallBackGlobalPointCloudMap, this);
    
    if(param_b_semantic_map_use_ == true)   
    {
        rossub_local_point_cloud_map_ = nh.subscribe("/roi_transformedmap", 10, &LandmarkMatchingPF::CallBackGlobalSemanticPointCloudMap, this);
    }
    else                                    
    {
        rossub_local_point_cloud_map_ = nh.subscribe("/roi_transformedmap", 10, &LandmarkMatchingPF::CallBackLocalPointCloudMap, this);
    }

    rospub_estimated_pose_            = nh.advertise<geometry_msgs::PoseStamped>("/particleEstimatedPose", i_buffer_size);
    rospub_novatel_enu_pose_          = nh.advertise<geometry_msgs::PoseStamped>("/novatelENUPose", i_buffer_size);
    rospub_ublox_enu_pose_            = nh.advertise<geometry_msgs::PoseStamped>("/ubloxENUPose", i_buffer_size);
    rospub_transformed_point_cloud_   = nh.advertise<sensor_msgs::PointCloud2>("/transformed_point_cloud", i_buffer_size);
    rospub_cropped_point_cloud_       = nh.advertise<sensor_msgs::PointCloud2>("/cropped_point_cloud", i_buffer_size);
    rospub_particle_marker_array_     = nh.advertise<visualization_msgs::MarkerArray>("/particle_marker_array", i_buffer_size);

    // Get error pose between novatel and estimated pose
    rospub_error_pose_                = nh.advertise<geometry_msgs::Pose>("/error_pose", i_buffer_size);

    rospub_particle_pose_array_       = nh.advertise<geometry_msgs::PoseArray>("/particlePoseArray", i_buffer_size);
    rospub_gps_quality_              = nh.advertise<std_msgs::String>("/strmsg_gps_quality", i_buffer_size);
    rospub_matching_score_                = nh.advertise<std_msgs::String>("/laneScore", i_buffer_size);

    strmsg_result_path_ = "/home/soyeong/" + m_str_fileName;
    std::remove(strmsg_result_path_.c_str());
}

LandmarkMatchingPF::~LandmarkMatchingPF()
{
}

void LandmarkMatchingPF::GetParameter()
{
    ros::NodeHandle nh;

    nh.getParam("/landmark_matching_pf/m_str_fileName"  , m_str_fileName);
    
    nh.getParam("/landmark_matching_pf/param_b_localization_type_point_cloud_map_matching_"  , param_b_localization_type_point_cloud_map_matching_);
    nh.getParam("/landmark_matching_pf/param_b_localization_type_spc_image_matching_"       , param_b_localization_type_spc_image_matching_);
    nh.getParam("/landmark_matching_pf/param_b_localization_type_gps_"         , param_b_localization_type_gps_);
    nh.getParam("/landmark_matching_pf/param_b_refernce_gps_novatel_"          , param_b_refernce_gps_novatel_);

    nh.getParam("/landmark_matching_pf/param_i_num_particle"            , param_i_num_particle);
    nh.getParam("/landmark_matching_pf/param_d_input_vel_sigma_ms_"         , param_d_input_vel_sigma_ms_);
    nh.getParam("/landmark_matching_pf/param_d_input_yaw_rate_sigma_degs_"   , param_d_input_yaw_rate_sigma_degs_);
    param_d_input_yaw_rate_sigma_rads_ = param_d_input_yaw_rate_sigma_degs_ * M_PI/180.0;

    nh.getParam("/landmark_matching_pf/param_d_input_acc_sigma_mss_"        , param_d_input_acc_sigma_mss_);
    nh.getParam("/landmark_matching_pf/param_d_east_sigma_m_"              , param_d_east_sigma_m_);
    nh.getParam("/landmark_matching_pf/param_d_north_sigma_m_"              , param_d_north_sigma_m_);
    nh.getParam("/landmark_matching_pf/param_d_heading_sigma_deg_"         , param_d_heading_sigma_deg_);
    param_d_heading_sigma_rad_ = param_d_heading_sigma_deg_*M_PI/180.0;

    nh.getParam("/landmark_matching_pf/param_d_resampling_coefficent_"         , param_d_resampling_coefficent_);
    nh.getParam("/landmark_matching_pf/param_d_gnss_radius_boundary_m_"   , param_d_gnss_radius_boundary_m_);
    nh.getParam("/landmark_matching_pf/param_i_num_map_matching_random_sample_point_"        , param_i_num_map_matching_random_sample_point_);
    nh.getParam("/landmark_matching_pf/param_d_map_noise_measure_noise_sigma_m_" , param_d_map_noise_measure_noise_sigma_m_);
    nh.getParam("/landmark_matching_pf/param_d_map_sigma_" , param_d_map_sigma_);

    nh.getParam("/landmark_matching_pf/param_d_lane_min_x_roi_m_"         , param_d_lane_min_x_roi_m_);
    nh.getParam("/landmark_matching_pf/param_d_lane_max_x_roi_m_"         , param_d_lane_max_x_roi_m_);
    nh.getParam("/landmark_matching_pf/param_d_lane_min_y_roi_m_"         , param_d_lane_min_y_roi_m_);
    nh.getParam("/landmark_matching_pf/param_d_lane_max_y_roi_m_"         , param_d_lane_max_y_roi_m_);

    nh.getParam("/landmark_matching_pf/param_d_building_min_x_roi_m_"         , param_d_building_min_x_roi_m_);
    nh.getParam("/landmark_matching_pf/param_d_building_max_x_roi_m_"         , param_d_building_max_x_roi_m_);
    nh.getParam("/landmark_matching_pf/param_d_building_min_y_roi_m_"         , param_d_building_min_y_roi_m_);
    nh.getParam("/landmark_matching_pf/param_d_building_max_y_roi_m_"         , param_d_building_max_y_roi_m_);

    nh.getParam("/landmark_matching_pf/param_d_fence_min_x_roi_m_"         , param_d_fence_min_x_roi_m_);
    nh.getParam("/landmark_matching_pf/param_d_fence_max_x_roi_m_"         , param_d_fence_max_x_roi_m_);
    nh.getParam("/landmark_matching_pf/param_d_fence_min_y_roi_m_"         , param_d_fence_min_y_roi_m_);
    nh.getParam("/landmark_matching_pf/param_d_fence_max_y_roi_m_"         , param_d_fence_max_y_roi_m_);

    nh.getParam("/landmark_matching_pf/param_d_tunnel_min_x_roi_m_"         , param_d_tunnel_min_x_roi_m_);
    nh.getParam("/landmark_matching_pf/param_d_tunnel_max_x_roi_m_"         , param_d_tunnel_max_x_roi_m_);
    nh.getParam("/landmark_matching_pf/param_d_tunnel_min_y_roi_m_"         , param_d_tunnel_min_y_roi_m_);
    nh.getParam("/landmark_matching_pf/param_d_tunnel_max_y_roi_m_"         , param_d_tunnel_max_y_roi_m_);

    nh.getParam("/landmark_matching_pf/param_d_traffic_min_x_roi_m_"         , param_d_traffic_min_x_roi_m_);
    nh.getParam("/landmark_matching_pf/param_d_traffic_max_x_roi_m_"         , param_d_traffic_max_x_roi_m_);
    nh.getParam("/landmark_matching_pf/param_d_traffic_min_y_roi_m_"         , param_d_traffic_min_y_roi_m_);
    nh.getParam("/landmark_matching_pf/param_d_traffic_max_y_roi_m_"         , param_d_pole_max_y_roi_m_);

    nh.getParam("/landmark_matching_pf/param_d_pole_min_x_roi_m_"         , param_d_pole_min_x_roi_m_);
    nh.getParam("/landmark_matching_pf/param_d_pole_max_x_roi_m_"         , param_d_pole_max_x_roi_m_);
    nh.getParam("/landmark_matching_pf/param_d_pole_min_y_roi_m_"         , param_d_pole_min_y_roi_m_);
    nh.getParam("/landmark_matching_pf/param_d_pole_max_y_roi_m_"         , param_d_pole_max_y_roi_m_);

    nh.getParam("/landmark_matching_pf/param_d_extrinsic_calibration_roll_deg_"         , param_d_extrinsic_calibration_roll_deg_);
    nh.getParam("/landmark_matching_pf/param_d_extrinsic_calibration_pitch_deg_"         , param_d_extrinsic_calibration_pitch_deg_);
    nh.getParam("/landmark_matching_pf/param_d_extrinsic_calibration_yaw_deg_"         , param_d_extrinsic_calibration_yaw_deg_);
    nh.getParam("/landmark_matching_pf/param_d_extrinsic_calibration_x_m_"         , param_d_extrinsic_calibration_x_m_);
    nh.getParam("/landmark_matching_pf/param_d_extrinsic_calibration_y_m_"         , param_d_extrinsic_calibration_y_m_);
    nh.getParam("/landmark_matching_pf/param_d_extrinsic_calibration_z_m_"         , param_d_extrinsic_calibration_z_m_);

    nh.getParam("/landmark_matching_pf/param_d_imu_height_m_"         , param_d_imu_height_m_);

    nh.getParam("/landmark_matching_pf/param_d_pole_likelihood_thresholde_"         , param_d_pole_likelihood_thresholde_);
    nh.getParam("/landmark_matching_pf/param_d_traffic_sign_likelihood_thresholde_"  , param_d_traffic_sign_likelihood_thresholde_);
    nh.getParam("/landmark_matching_pf/param_d_traffic_light_likelihood_threshold_" , param_d_traffic_light_likelihood_threshold_);
    nh.getParam("/landmark_matching_pf/param_d_tunnel_fan_likelihood_threshold_"    , param_d_tunnel_fan_likelihood_threshold_);
    nh.getParam("/landmark_matching_pf/param_d_tunnel_light_likelihood_threshold_"  , param_d_tunnel_light_likelihood_threshold_);
    nh.getParam("/landmark_matching_pf/param_d_tunnel_hydrant_likelihood_threshold_", param_d_tunnel_hydrant_likelihood_threshold_);
    nh.getParam("/landmark_matching_pf/param_d_building_likelihood_threshold_"     , param_d_building_likelihood_threshold_);
    nh.getParam("/landmark_matching_pf/param_d_lane_marking_likelihood_threshold_"  , param_d_lane_marking_likelihood_threshold_);
    nh.getParam("/landmark_matching_pf/param_d_fence_likelihood_threshold_"        , param_d_fence_likelihood_threshold_);

    nh.getParam("/landmark_matching_pf/param_d_gnss_position_quality_threshold_m_", param_d_gnss_position_quality_threshold_m_);
    nh.getParam("/landmark_matching_pf/param_d_gnss_fault_position_threshold_m_", param_d_gnss_fault_position_threshold_m_);
    nh.getParam("/landmark_matching_pf/param_d_gnss_fault_heading_threshold_deg_", param_d_gnss_fault_heading_threshold_deg_);

    nh.getParam("/landmark_matching_pf/param_b_semantic_map_use_", param_b_semantic_map_use_); // true: Sementic map , false: normal map

    nh.getParam("/landmark_matching_pf/param_i_num_sampling_building_", param_i_num_sampling_building_); 
    nh.getParam("/landmark_matching_pf/param_i_num_sampling_fence_", param_i_num_sampling_fence_); 
    nh.getParam("/landmark_matching_pf/param_i_num_sampling_lane_marking_", param_i_num_sampling_lane_marking_); 
    nh.getParam("/landmark_matching_pf/param_i_num_sampling_trunk_", param_i_num_sampling_trunk_); 
    nh.getParam("/landmark_matching_pf/param_i_num_sampling_pole_", param_i_num_sampling_pole_); 
    nh.getParam("/landmark_matching_pf/param_i_num_sampling_traffic_sign_", param_i_num_sampling_traffic_sign_); 
    nh.getParam("/landmark_matching_pf/param_i_num_sampling_traffic_light_", param_i_num_sampling_traffic_light_); 
    nh.getParam("/landmark_matching_pf/param_i_num_sampling_tunnel_fan_", param_i_num_sampling_tunnel_fan_); 
    nh.getParam("/landmark_matching_pf/param_i_num_sampling_tunnel_light_", param_i_num_sampling_tunnel_light_); 
    nh.getParam("/landmark_matching_pf/param_i_num_sampling_tunnel_hydrant_", param_i_num_sampling_tunnel_hydrant_); 

    nh.getParam("/landmark_matching_pf/param_i_visualize_particle_index_", param_i_visualize_particle_index_); 

    nh.getParam("/landmark_matching_pf/param_b_polygon_view_on_", param_b_polygon_view_on_); 
    nh.getParam("/landmark_matching_pf/param_b_segmented_view_on_", param_b_segmented_view_on_); 
}

void LandmarkMatchingPF::Initialization()
{
    int i_init_loop_frequency = 20;
    ros::Rate rosrate_loop_rate(i_init_loop_frequency);

    // PF Variable initialization
    egmat_timestamped_particle_state_                   = new Eigen::MatrixXd (Eigen::MatrixXd(param_i_num_particle, i_num_of_state_));
    egmat_timestamped_particle_state_resampling_        = new Eigen::MatrixXd (Eigen::MatrixXd(param_i_num_particle, i_num_of_state_));
    egmat_particle_weight_                              = new Eigen::MatrixXd (Eigen::MatrixXd(param_i_num_particle, 1));
    egmat_gps_particle_likelihood_                      = new Eigen::MatrixXd (Eigen::MatrixXd(param_i_num_particle, 1));
    egmat_map_matching_particle_likelihood_             = new Eigen::MatrixXd (Eigen::MatrixXd(param_i_num_particle, 1));

    egmat_building_particle_likelihood_                 = new Eigen::MatrixXd (Eigen::MatrixXd(param_i_num_particle, 1));
    egmat_lane_marking_particle_likelihood_             = new Eigen::MatrixXd (Eigen::MatrixXd(param_i_num_particle, 1));
    egmat_fence_particle_likelihood_                    = new Eigen::MatrixXd (Eigen::MatrixXd(param_i_num_particle, 1));
    egmat_pole_particle_likelihood_                     = new Eigen::MatrixXd (Eigen::MatrixXd(param_i_num_particle, 1));
    egmat_traffic_sign_particle_likelihood_             = new Eigen::MatrixXd (Eigen::MatrixXd(param_i_num_particle, 1));
    egmat_traffic_light_particle_likelihood_            = new Eigen::MatrixXd (Eigen::MatrixXd(param_i_num_particle, 1));
    egmat_tunnel_fan_particle_likeilhood_               = new Eigen::MatrixXd (Eigen::MatrixXd(param_i_num_particle, 1));
    egmat_tunnel_light_particle_likelihood_             = new Eigen::MatrixXd (Eigen::MatrixXd(param_i_num_particle, 1));
    egmat_tunnel_hydrant_particle_likelihood_           = new Eigen::MatrixXd (Eigen::MatrixXd(param_i_num_particle, 1));

    egmat_estimated_state_                              = new Eigen::MatrixXd (Eigen::MatrixXd(1, i_num_of_state_));
    egmat_estimated_sigma_                              = new Eigen::MatrixXd (Eigen::MatrixXd(1, i_num_of_state_));

    while(!checker_b_reference_pose_data_init_ && ros::ok())
    {
        ROS_WARN_STREAM("Initializing...");
        if(checker_b_gnss_data_init_ && checker_b_is_first_IMU_step_ && checker_b_map_data_init_)
        {
            checker_b_reference_pose_data_init_ = true;
            checker_b_initialization_complete_    = true;
            ROS_WARN_STREAM("Initialized !!!");
            return;
        }
        ros::spinOnce();
        rosrate_loop_rate.sleep();
    }
}

void LandmarkMatchingPF::Run(){

    if(!checker_b_gnss_data_init_ || !checker_b_is_first_IMU_step_ || !checker_b_map_data_init_){
        m_init_time = std::chrono::high_resolution_clock::now();
        return;
    }

    geometry_msgs::PoseStamped psstp_estimated_pose;

    if(checker_b_imu_data_exist_){
        // prediction
        imu_imu_data_.vel_ms = gps_gps_data_.vel_ms;
        auto t1 = std::chrono::high_resolution_clock::now();
        PFPrediction();
        auto t2 = std::chrono::high_resolution_clock::now();
        auto duration_prediction = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();

        t1 = std::chrono::high_resolution_clock::now();
        PFRepresentativePoseExtraction();
        t2 = std::chrono::high_resolution_clock::now();
        auto duration_density = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();
        
        t1 = std::chrono::high_resolution_clock::now();
        PFResampling();
        t2 = std::chrono::high_resolution_clock::now();
        auto duration_resampling = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();

        std::cout <<"[COMPUTATION TIME] PF PREDICTION TIME: "        <<duration_prediction/1e6<<"sec"<< std::endl;
        std::cout <<"[COMPUTATION TIME] PF DENSITY EXTRACTION TIME: "<<duration_density   /1e6<<"sec"<< std::endl;
        std::cout <<"[COMPUTATION TIME] PF RESAMPLING TIME: "        <<duration_resampling/1e6<<"sec"<< std::endl;

        checker_b_imu_data_exist_ = false;
    }

    // Measurement Update
    if(param_b_localization_type_gps_)
    {
        if(checker_b_gps_data_exist_ || checker_b_ublox_data_exist_){
            // weight update
            auto t1 = std::chrono::high_resolution_clock::now();
            GPSMeasurementUpdate();
            
            auto t2 = std::chrono::high_resolution_clock::now();
            auto duration_gpcm_gps = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();
            std::cout <<"[COMPUTATION TIME] PF MAP MATCHING WITH GPS TIME: "<<duration_gpcm_gps/1e6<<"sec"<< std::endl;

            checker_b_gps_data_exist_ = false;
            checker_b_ublox_data_exist_ = false;
        }
    }
    
    if(param_b_localization_type_spc_image_matching_)
    {
        std::vector<cv::Point2d> vec_ref_position_point_projected_image;

        auto t1 = std::chrono::high_resolution_clock::now();
        if((checker_b_image_contour_data_exist_ || checker_b_segmented_image_data_exist_)  && fabs(gps_gps_data_.vel_ms*3.6) > 3.0) 
        {
            param_b_refernce_gps_novatel_ = false;

            ParticlePointsTransformation();
            PointCloud2DProjection(pcptr_input_local_point_cloud_map_, egmat_lidar_vehicle_transform_, vec_ref_position_point_projected_image); // for visualization

            ImageSPCMeasurementUpdate();

            //***********************for ref tf_transformation***************************
            std::vector<cv::Point2d> vec_par_position_point_projectd_image;

            if(checker_b_image_contour_data_exist_ && param_b_polygon_view_on_)
                ReferenceImagePointAndCheckInsideViewer();
                checker_b_image_contour_data_exist_ = false;
                
            if(checker_b_segmented_image_data_exist_ && param_b_segmented_view_on_)
                ReferenceSegmentedImagePointAndCheckInsideViewer();
                checker_b_segmented_image_data_exist_ = false;

            if(param_i_visualize_particle_index_ >= 0 && param_i_visualize_particle_index_<param_i_num_particle)
            {
                PointCloud2DProjection(pcptr_input_local_point_cloud_map_, vec_ref_par_transform_[param_i_visualize_particle_index_], vec_par_position_point_projectd_image);
                ParticleImagePointViewer(vec_par_position_point_projectd_image, param_i_visualize_particle_index_);
            }
        }

        if(checker_b_image_data_exist_)
        {
            ReferenceImagePointViewer(vec_ref_position_point_projected_image);
            checker_b_image_data_exist_ = false;
        }
        auto t2 = std::chrono::high_resolution_clock::now();
        auto duration_spc_image = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();
        std::cout <<"[COMPUTATION TIME] IMAGE SPC MATCHING TIME: "<<duration_spc_image/1e6<<"sec"<< std::endl;
    }

    if(param_b_localization_type_point_cloud_map_matching_)
    {
        if(pcptr_input_point_cloud_ != NULL)
        {
            auto t1 = std::chrono::high_resolution_clock::now();
            MapMatchingMeasurementUpdate();
            auto t2 = std::chrono::high_resolution_clock::now();
            auto duration_gpcm_only = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();
            std::cout <<"[COMPUTATION TIME] PF MAP MATCHING ONLY TIME: "<<duration_gpcm_only/1e6<<"sec"<< std::endl;
        }
    }

    //Publish Particle in Rviz
    PublishParticlePoseArray();
    
    psstp_estimated_pose.header.frame_id   = "map";
    psstp_estimated_pose.header.stamp.sec  = imu_imu_data_.sec;
    psstp_estimated_pose.header.stamp.nsec = imu_imu_data_.nsec;

    psstp_estimated_pose.pose.position.x = egmat_estimated_state_->coeff(0, 0);
    psstp_estimated_pose.pose.position.y = egmat_estimated_state_->coeff(0, 1);
    psstp_estimated_pose.pose.position.z = d_map_height_;
    
    tf::Quaternion tfquat_quaternion;
    tfquat_quaternion.setRPY(d_map_roll_, d_map_pitch_, egmat_estimated_state_->coeff(0, 2));

    geometry_msgs::Quaternion gquat_quaternion;
    tf::quaternionTFToMsg(tfquat_quaternion, gquat_quaternion);

    psstp_estimated_pose.pose.orientation = gquat_quaternion;

    rospub_estimated_pose_.publish(psstp_estimated_pose);
    
    // Visualize GPS Sensor Data
    geometry_msgs::PoseStamped psstp_novatel_enu_pose;
    psstp_novatel_enu_pose = ConvertToMapFrame(gps_gps_data_.latitude, gps_gps_data_.longitude, gps_gps_data_.height);

    geometry_msgs::PoseStamped psstp_ublox_enu_pose;
    psstp_ublox_enu_pose = ConvertToMapFrame(gps_ublox_data_.latitude, gps_ublox_data_.longitude, gps_ublox_data_.height);

    rospub_novatel_enu_pose_.publish(psstp_novatel_enu_pose);
    rospub_ublox_enu_pose_.publish(psstp_ublox_enu_pose);

    //Get error plot
    geometry_msgs::Pose errorPose;
    errorPose.position.x = psstp_estimated_pose.pose.position.x - psstp_novatel_enu_pose.pose.position.x;
    errorPose.position.y = psstp_estimated_pose.pose.position.y - psstp_novatel_enu_pose.pose.position.y;
    rospub_error_pose_.publish(errorPose);
}


// CALLBACK FUNCTION
void LandmarkMatchingPF::CallBackGlobalSemanticPointCloudMap(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcptr_point_cloud_input       (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*msg, *pcptr_point_cloud_input);

    int i_input_point_size = pcptr_point_cloud_input->points.size();

    pcptr_input_local_point_cloud_map_->points.clear();


    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcptr_cropped_semantic_point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    for(auto& point : pcptr_point_cloud_input->points)
    {
        if(point.x>0. && fabs(point.y)<10.0)
        {
            pcptr_cropped_semantic_point_cloud->points.push_back(point);
        }
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcptr_building_point_cloud       (new pcl::PointCloud<pcl::PointXYZRGB>); // rgb 255 200   0, 7 249 184
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcptr_fence_point_cloud          (new pcl::PointCloud<pcl::PointXYZRGB>); // rgb 255 120  50
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcptr_lane_marking_point_cloud   (new pcl::PointCloud<pcl::PointXYZRGB>); // rgb 150 255 170
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcptr_trunk_point_cloud          (new pcl::PointCloud<pcl::PointXYZRGB>); // rgb 135  60   0
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcptr_pole_point_cloud           (new pcl::PointCloud<pcl::PointXYZRGB>); // rgb 255 240 150
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcptr_traffic_sign_point_cloud   (new pcl::PointCloud<pcl::PointXYZRGB>); // rgb 255   0   0
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcptr_traffic_light_point_cloud  (new pcl::PointCloud<pcl::PointXYZRGB>); // rgb   0   0 255
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcptr_tunnel_fan_point_cluod     (new pcl::PointCloud<pcl::PointXYZRGB>); // rgb   0 100  80
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcptr_tunnel_light_point_cloud   (new pcl::PointCloud<pcl::PointXYZRGB>); // rgb 174  95 137
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcptr_tunnel_hydrant_point_cloud (new pcl::PointCloud<pcl::PointXYZRGB>); // rgb 171   0 255

    for(auto& rgb_points : pcptr_cropped_semantic_point_cloud->points){
        rgb_points.r = (int)rgb_points.r;
        rgb_points.g = (int)rgb_points.g;
        rgb_points.b = (int)rgb_points.b;

        if(rgb_points.r == 255 && rgb_points.g == 200 && rgb_points.b ==   0){ // str_building

            if(rgb_points.x>param_d_building_min_x_roi_m_ && rgb_points.x<param_d_building_max_x_roi_m_ 
                && rgb_points.y>param_d_building_min_y_roi_m_ && rgb_points.y<param_d_building_max_y_roi_m_)
            {
                pcptr_building_point_cloud->points.push_back(rgb_points);
            }
        }
        else if(rgb_points.r ==   7 && rgb_points.g == 249 && rgb_points.b == 184){ // str_building

            if(rgb_points.x>param_d_building_min_x_roi_m_ && rgb_points.x<param_d_building_max_x_roi_m_ 
                && rgb_points.y>param_d_building_min_y_roi_m_ && rgb_points.y<param_d_building_max_y_roi_m_)
            {
                pcptr_building_point_cloud->points.push_back(rgb_points);
            }
        }
        else if(rgb_points.r == 255 && rgb_points.g == 120 && rgb_points.b ==  50){ // str_fence

            if(rgb_points.x>param_d_fence_min_x_roi_m_ && rgb_points.x<param_d_fence_max_x_roi_m_ 
                && rgb_points.y>param_d_fence_min_y_roi_m_ && rgb_points.y<param_d_fence_max_y_roi_m_)
            {
                pcptr_fence_point_cloud->points.push_back(rgb_points);
            }
        }
        else if(rgb_points.r == 150 && rgb_points.g == 255 && rgb_points.b == 170){ // lane marking

            if(rgb_points.x>param_d_lane_min_x_roi_m_ && rgb_points.x<param_d_lane_max_x_roi_m_ 
                && rgb_points.y>param_d_lane_min_y_roi_m_ && rgb_points.y<param_d_lane_max_y_roi_m_)
            {
                pcptr_lane_marking_point_cloud->points.push_back(rgb_points);
            }
        }
        else if(rgb_points.r == 135 && rgb_points.g ==  60 && rgb_points.b ==   0){ // trunk

            if(rgb_points.x>param_d_pole_min_x_roi_m_ && rgb_points.x<param_d_pole_max_x_roi_m_ 
                && rgb_points.y>param_d_pole_min_y_roi_m_ && rgb_points.y<param_d_pole_max_y_roi_m_)
            {
                pcptr_pole_point_cloud->points.push_back(rgb_points);
            }
        }
        else if(rgb_points.r == 255 && rgb_points.g == 240 && rgb_points.b == 150){ // str_pole

            if(rgb_points.x>param_d_pole_min_x_roi_m_ && rgb_points.x<param_d_pole_max_x_roi_m_ 
                && rgb_points.y>param_d_pole_min_y_roi_m_ && rgb_points.y<param_d_pole_max_y_roi_m_)
            {
                pcptr_pole_point_cloud->points.push_back(rgb_points);
            }
        }
        else if(rgb_points.r == 255 && rgb_points.g ==   0 && rgb_points.b ==   0){ // traffic sign

            if(rgb_points.x>param_d_traffic_min_x_roi_m_ && rgb_points.x<param_d_traffic_max_x_roi_m_ 
                && rgb_points.y>param_d_traffic_min_y_roi_m_ && rgb_points.y<param_d_traffic_max_y_roi_m_)
            {
                pcptr_traffic_sign_point_cloud->points.push_back(rgb_points);
            }
        }
        else if(rgb_points.r ==   0 && rgb_points.g ==   0 && rgb_points.b == 255){ // traffic light

            if(rgb_points.x>param_d_traffic_min_x_roi_m_ && rgb_points.x<param_d_traffic_max_x_roi_m_ 
                && rgb_points.y>param_d_traffic_min_y_roi_m_ && rgb_points.y<param_d_traffic_max_y_roi_m_)
            {
                pcptr_traffic_light_point_cloud->points.push_back(rgb_points);
            }
        }
        else if(rgb_points.r ==   0 && rgb_points.g == 100 && rgb_points.b ==  80){ // tunnel fan

            if(rgb_points.x>param_d_tunnel_min_x_roi_m_ && rgb_points.x<param_d_tunnel_max_x_roi_m_ 
                && rgb_points.y>param_d_tunnel_min_y_roi_m_ && rgb_points.y<param_d_tunnel_max_y_roi_m_)
            {
                pcptr_tunnel_fan_point_cluod->points.push_back(rgb_points);
            }
        }
        else if(rgb_points.r == 174 && rgb_points.g ==  95 && rgb_points.b == 137){ // tunnel light

            if(rgb_points.x>param_d_tunnel_min_x_roi_m_ && rgb_points.x<param_d_tunnel_max_x_roi_m_ 
                && rgb_points.y>param_d_tunnel_min_y_roi_m_ && rgb_points.y<param_d_tunnel_max_y_roi_m_)
            {
                pcptr_tunnel_light_point_cloud->points.push_back(rgb_points);
            }
        }
        else if(rgb_points.r == 171 && rgb_points.g ==   0 && rgb_points.b == 255){ // tunnel hydrant

            if(rgb_points.x>param_d_tunnel_min_x_roi_m_ && rgb_points.x<param_d_tunnel_max_x_roi_m_ 
                && rgb_points.y>param_d_tunnel_min_y_roi_m_ && rgb_points.y<param_d_tunnel_max_y_roi_m_)
            {
                pcptr_tunnel_hydrant_point_cloud->points.push_back(rgb_points);
            }
        }
    }

    pcl::RandomSample<pcl::PointXYZRGB> pcrandom_sampling;
    if(param_i_num_sampling_building_ > 0)
    {
        pcrandom_sampling.setSample(param_i_num_sampling_building_);
        pcrandom_sampling.setInputCloud(pcptr_building_point_cloud);
        pcrandom_sampling.filter(*pcptr_building_sampled_point_cloud_);
    }
    else
    {
        pcptr_building_sampled_point_cloud_->clear();
    }
    

    if(param_i_num_sampling_fence_ > 0)
    {
        pcrandom_sampling.setSample(param_i_num_sampling_fence_);
        pcrandom_sampling.setInputCloud(pcptr_fence_point_cloud);
        pcrandom_sampling.filter(*pcptr_fence_sampled_point_cloud_);
    }
    else
    {
        pcptr_fence_sampled_point_cloud_->clear();
    }

    if(param_i_num_sampling_lane_marking_ > 0)
    {
        pcrandom_sampling.setSample(param_i_num_sampling_lane_marking_);
        pcrandom_sampling.setInputCloud(pcptr_lane_marking_point_cloud);
        pcrandom_sampling.filter(*pcptr_lane_marking_sampled_point_cloud_);
    }
    else
    {
        pcptr_lane_marking_sampled_point_cloud_->clear();
    }

    if(param_i_num_sampling_trunk_ > 0)
    {
        pcrandom_sampling.setSample(param_i_num_sampling_trunk_);
        pcrandom_sampling.setInputCloud(pcptr_trunk_point_cloud);
        pcrandom_sampling.filter(*pcptr_trunk_sampled_point_cloud_);
    }
    else
    {
        pcptr_trunk_sampled_point_cloud_->clear();
    }

    if(param_i_num_sampling_pole_ > 0)
    {
        pcrandom_sampling.setSample(param_i_num_sampling_pole_);
        pcrandom_sampling.setInputCloud(pcptr_pole_point_cloud);
        pcrandom_sampling.filter(*pcptr_pole_sampled_point_cloud_);
    }
    else
    {
        pcptr_pole_sampled_point_cloud_->clear();
    }

    if(param_i_num_sampling_traffic_sign_ > 0)
    {
        pcrandom_sampling.setSample(param_i_num_sampling_traffic_sign_);
        pcrandom_sampling.setInputCloud(pcptr_traffic_sign_point_cloud);
        pcrandom_sampling.filter(*pcptr_traffic_sign_sampled_point_cloud_);
    }
    else
    {
        pcptr_traffic_sign_sampled_point_cloud_->clear();
    }

    if(param_i_num_sampling_traffic_light_ > 0)
    {
        pcrandom_sampling.setSample(param_i_num_sampling_traffic_light_);
        pcrandom_sampling.setInputCloud(pcptr_traffic_light_point_cloud);
        pcrandom_sampling.filter(*pcptr_traffic_light_sampled_point_cloud_);
    }
    else
    {
        pcptr_traffic_light_sampled_point_cloud_->clear();
    }

    if(param_i_num_sampling_tunnel_fan_ > 0)
    {
        pcrandom_sampling.setSample(param_i_num_sampling_tunnel_fan_);
        pcrandom_sampling.setInputCloud(pcptr_tunnel_fan_point_cluod);
        pcrandom_sampling.filter(*pcptr_tunnel_fan_sampled_point_cloud_);
    }
    else
    {
        pcptr_tunnel_fan_sampled_point_cloud_->clear();
    }

    if(param_i_num_sampling_tunnel_light_ > 0)
    {
        pcrandom_sampling.setSample(param_i_num_sampling_tunnel_light_);
        pcrandom_sampling.setInputCloud(pcptr_tunnel_light_point_cloud);
        pcrandom_sampling.filter(*pcptr_tunnel_light_sampled_point_cloud_);
    }
    else
    {
        pcptr_tunnel_light_sampled_point_cloud_->clear();
    }

    if(param_i_num_sampling_tunnel_hydrant_ > 0)
    {
        pcrandom_sampling.setSample(param_i_num_sampling_tunnel_hydrant_);
        pcrandom_sampling.setInputCloud(pcptr_tunnel_hydrant_point_cloud);
        pcrandom_sampling.filter(*pcptr_tunnel_hydrant_sampled_point_cloud_);
    }
    else
    {
        pcptr_tunnel_hydrant_sampled_point_cloud_->clear();
    }

    *pcptr_input_local_point_cloud_map_ += *pcptr_building_sampled_point_cloud_;
    *pcptr_input_local_point_cloud_map_ += *pcptr_fence_sampled_point_cloud_;
    *pcptr_input_local_point_cloud_map_ += *pcptr_lane_marking_sampled_point_cloud_;
    *pcptr_input_local_point_cloud_map_ += *pcptr_trunk_sampled_point_cloud_;
    *pcptr_input_local_point_cloud_map_ += *pcptr_pole_sampled_point_cloud_;
    *pcptr_input_local_point_cloud_map_ += *pcptr_traffic_sign_sampled_point_cloud_;
    *pcptr_input_local_point_cloud_map_ += *pcptr_traffic_light_sampled_point_cloud_;
    *pcptr_input_local_point_cloud_map_ += *pcptr_tunnel_fan_sampled_point_cloud_;
    *pcptr_input_local_point_cloud_map_ += *pcptr_tunnel_light_sampled_point_cloud_;
    *pcptr_input_local_point_cloud_map_ += *pcptr_tunnel_hydrant_sampled_point_cloud_;
}

void LandmarkMatchingPF::CallbackSegmentedImage(const sensor_msgs::Image::ConstPtr& msg)
{
    if(!msg->data.empty())
    {
        checker_b_segmented_image_data_exist_ = true;
        cv_bridge::CvImagePtr cvptr_image;
        try
        {
            cvptr_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        
        cv::Mat cvmat_image = cvptr_image->image;
        cvmat_segmented_image_ = cvmat_image;
    }
}

void LandmarkMatchingPF::CallbackImageDetectionLabel(const landmark_matching_localization::Labels::ConstPtr &msg)
{
    if(!msg->labels.empty())
    {
        checker_b_image_contour_data_exist_ = true;

        std::vector<std::vector<cv::Point>> vec_image_contour_pole;
        std::vector<std::vector<cv::Point>> vec_image_contour_traffic_sign;
        std::vector<std::vector<cv::Point>> vec_image_contour_traffic_light;
        std::vector<std::vector<cv::Point>> vec_image_contour_tunnel_fan;
        std::vector<std::vector<cv::Point>> vec_image_contour_tunnel_light;
        std::vector<std::vector<cv::Point>> vec_image_contour_tunnel_hydrant;

        for(auto idxLabel = 0; idxLabel < msg->labels.size(); idxLabel++)
        {
            landmark_matching_localization::Label spcimagematching_image_label = msg->labels[idxLabel];
            std::vector<cv::Point> vec_contour;

            for(auto idxContour = 0; idxContour < spcimagematching_image_label.contour.size(); idxContour++)
            {
                cv::Point2d cvp_contour_points;
                cvp_contour_points.x = spcimagematching_image_label.contour[idxContour].x;
                cvp_contour_points.y = spcimagematching_image_label.contour[idxContour].y;
                vec_contour.push_back(cvp_contour_points);
            }

            switch((int)spcimagematching_image_label.category)
            {
                case 1:
                    vec_image_contour_pole.push_back(vec_contour);
                    break;
                case 2:
                    vec_image_contour_traffic_sign.push_back(vec_contour);
                    break;
                case 3:
                    vec_image_contour_traffic_light.push_back(vec_contour);
                    break;
                case 4:
                    vec_image_contour_tunnel_fan.push_back(vec_contour);
                    break;
                case 5:
                    vec_image_contour_tunnel_light.push_back(vec_contour);
                    break;
                case 6:
                    vec_image_contour_tunnel_hydrant.push_back(vec_contour);
                    break;
            }
        }

        vec_image_contour_point_pole_ = vec_image_contour_pole; // str_pole
        vec_image_contour_point_traffic_sign_ = vec_image_contour_traffic_sign; // traffic sign
        vec_image_contour_point_traffic_light_ = vec_image_contour_traffic_light; // traffic light
        vec_image_contour_point_tunnel_fan_ = vec_image_contour_tunnel_fan; // tunnel fan
        vec_image_contour_point_tunnel_light_ = vec_image_contour_tunnel_light; // tunnel fire hydrant
        vec_image_contour_point_tunnel_hydrant = vec_image_contour_tunnel_hydrant; // tunnel emergency light
    }
}

void LandmarkMatchingPF::CallBackGlobalPointCloudMap(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    if(kdtree_global_map_!=NULL)
    {
        return;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcptr_point_cloud_input(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcptr_point_cloud_input->clear();

    pcl::fromROSMsg(*msg, *pcptr_point_cloud_input);

    pcl::KdTreeFLANN<pcl::PointXYZRGB>::Ptr pcptr_map_kdtree(new pcl::KdTreeFLANN<pcl::PointXYZRGB>);
    pcptr_map_kdtree->setInputCloud(pcptr_point_cloud_input);
    kdtree_global_map_ = pcptr_map_kdtree;
    checker_b_map_data_init_ = true;
}

void LandmarkMatchingPF::CallBackLocalPointCloudMap(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcptr_point_cloud_input       (new pcl::PointCloud<pcl::PointXYZRGB>);
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclRandomPointCloudInput (new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::fromROSMsg(*msg, *pcptr_point_cloud_input);

    pcptr_input_local_point_cloud_map_ = pcptr_point_cloud_input;
}

void LandmarkMatchingPF::CallBackVelodyne(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    sensor_msgs::PointCloud2 pc2_output_point_cloud;

    static tf::TransformListener tflistener_ego_velodyne;
    tf::StampedTransform transform_tf;
    try{
        tflistener_ego_velodyne.lookupTransform("map", msg->header.frame_id, ros::Time(0), transform_tf);
    }
    catch(tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
    }
    pcl_ros::transformPointCloud("map", transform_tf, *msg, pc2_output_point_cloud);
    rospub_transformed_point_cloud_.publish(pc2_output_point_cloud);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcptr_point_cloud_input(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcptr_point_cloud_input->clear();

    pcl::fromROSMsg(*msg, *pcptr_point_cloud_input);
    pcptr_input_point_cloud_ = pcptr_point_cloud_input;
}

void LandmarkMatchingPF::CallBackMapHeight(const geometry_msgs::Pose::ConstPtr &msg)
{
    d_map_height_ = msg->position.z + param_d_imu_height_m_;
}

void LandmarkMatchingPF::CallbackImage(const sensor_msgs::ImageConstPtr& input)
{
    // Point2Image->SetInputImage(input);
    checker_b_image_data_exist_ = true;
    imgmsg_input_image_ = input;
    cvptr_input_image_ = cv_bridge::toCvCopy(imgmsg_input_image_, sensor_msgs::image_encodings::BGR8);

    cv::Mat cvp_image = cvptr_input_image_->image;
    d_input_image_size_ = cvp_image.cols * cvp_image.rows;
}

void LandmarkMatchingPF::CallBackNovatelINSPVAX(const novatel_oem7_msgs::INSPVAX::ConstPtr &msg)
{
    double d_novatel_heading_bias_deg = 90.0;

    if(!checker_b_gnss_data_init_ && param_b_refernce_gps_novatel_ && (checker_b_map_data_init_ || param_b_localization_type_gps_)){
        gps_init_gps_data.timestamp     = (double)msg->header.stamp.sec * 1e6 + (double)msg->header.stamp.nsec / 1e3;
        gps_init_gps_data.sec           = (double)msg->header.stamp.sec;
        gps_init_gps_data.nsec          = (double)msg->header.stamp.nsec;
        gps_init_gps_data.latitude      = msg->latitude;
        gps_init_gps_data.longitude     = msg->longitude;
        gps_init_gps_data.height        = msg->height;
        gps_init_gps_data.yaw           = (-1. * msg->azimuth + d_novatel_heading_bias_deg) * M_PI /180.;
        gps_init_gps_data.vel_ms        = sqrt(pow(msg->north_velocity,2)+pow(msg->east_velocity,2));
        gps_init_gps_data.latitude_std  = msg->latitude_stdev;
        gps_init_gps_data.longitude_std = msg->longitude_stdev;
        gps_init_gps_data.height_std    = msg->height_stdev;

        psstp_init_gnss_enu_pose_ = ConvertToMapFrame(gps_init_gps_data.latitude, gps_init_gps_data.longitude, gps_init_gps_data.height);

        array_ego_vehicle_pose_[0] = psstp_init_gnss_enu_pose_.pose.position.x;
        array_ego_vehicle_pose_[1] = psstp_init_gnss_enu_pose_.pose.position.y;
        array_ego_vehicle_pose_[2] = gps_init_gps_data.yaw;

        PFParticleInit(array_ego_vehicle_pose_);
        
        checker_b_gnss_data_init_ = true;
    }
    checker_b_gps_data_exist_ = true;
    
    gps_gps_data_.timestamp = (double)msg->header.stamp.sec * 1e6 + (double)msg->header.stamp.nsec / 1e3;

    std::string str_gps_quality;

    switch(msg->pos_type.type)
    {
        case 16:
            gps_gps_data_.quality = 8;
            str_gps_quality = "SINGLE";
            break;
        case 17:
            gps_gps_data_.quality = 7;
            str_gps_quality = "PSRDIFF";
            break;
        case 34:
            gps_gps_data_.quality = 6;
            str_gps_quality = "NARROW_FLOAT";
            break;
        case 49:
            gps_gps_data_.quality = 5;
            str_gps_quality = "WIDE_INT";
            break;
        case 50:
            gps_gps_data_.quality = 4;
            str_gps_quality = "NARROW_INT";
            break;
        case 53:
            gps_gps_data_.quality = 3;
            str_gps_quality = "INS_PSRSP";
            break;
        case 54:
            gps_gps_data_.quality = 2;
            str_gps_quality = "INS_PSRDIFF";
            break;
        case 55:
            gps_gps_data_.quality = 1;
            str_gps_quality = "INS_RTKFLOAT";
            break;
        case 56:
            gps_gps_data_.quality = 0;
            str_gps_quality = "INS_RTKFIXED";
            break;
        default:
            gps_gps_data_.quality = -1;
            str_gps_quality = "INVALID QUALITY TYPE";
            break;
    }

    if(param_b_refernce_gps_novatel_)
    {
        std_msgs::String strmsg_gps_quality;
        strmsg_gps_quality.data = "Novatel GPS Quality: " + str_gps_quality;
        rospub_gps_quality_.publish(strmsg_gps_quality);
    }

    gps_gps_data_.latitude         = msg->latitude;
    gps_gps_data_.longitude        = msg->longitude;
    gps_gps_data_.height           = msg->height;

    gps_gps_data_.latitude_std     = msg->latitude_stdev;
    gps_gps_data_.longitude_std    = msg->longitude_stdev;
    gps_gps_data_.height_std       = msg->height_stdev;

    gps_gps_data_.yaw              = (-1. * msg->azimuth + d_novatel_heading_bias_deg) * M_PI /180.;
    gps_gps_data_.vel_ms           = sqrt(pow(msg->north_velocity,2)+pow(msg->east_velocity,2));

    psstp_gnss_enu_pose_ = ConvertToMapFrame(gps_gps_data_.latitude, gps_gps_data_.longitude, gps_gps_data_.height);

    double array_tmp_gps_state[3];
    array_tmp_gps_state[0] = psstp_gnss_enu_pose_.pose.position.x;
    array_tmp_gps_state[1] = psstp_gnss_enu_pose_.pose.position.y;
    array_tmp_gps_state[2] = gps_gps_data_.yaw;
        
    if(param_b_refernce_gps_novatel_)
    {
        if (PFFaultDetection())
        {
            PFParticleInit(array_tmp_gps_state);
            ROS_ERROR_STREAM("Particle Filter Fault Detection !!! ");
        }
    }
}

void LandmarkMatchingPF::CallBackNovatelCORRIMU(const novatel_oem7_msgs::CORRIMU::ConstPtr &msg)
{   
    checker_b_imu_data_exist_ = true;

    double d_timestamp = (double)msg->header.stamp.sec * 1e6 + (double)msg->header.stamp.nsec / 1e3;
    static double d_prev_timestamp = d_timestamp;
    if (checker_b_is_first_IMU_step_ == false)
    {
        checker_b_is_first_IMU_step_ = true;
        return;
    }

    double d_sample_time = (d_timestamp - d_prev_timestamp)/1e6;

    imu_imu_data_.timestamp      = d_timestamp;
    imu_imu_data_.sec            = (double)msg->header.stamp.sec;
    imu_imu_data_.nsec           = (double)msg->header.stamp.nsec;
    imu_imu_data_.yaw_rate_rads  = (msg->yaw_rate/d_sample_time)*10;
    imu_imu_data_.acc_mss        = msg->longitudinal_acc/d_sample_time;
    d_prev_timestamp = d_timestamp;
}

void LandmarkMatchingPF::CallBackUbloxHNRPVT(const ublox_msgs::HnrPVT::ConstPtr &msg)
{
    double d_ublox_heading_bias_deg = 90.0;

    if(!checker_b_gnss_data_init_ && !param_b_refernce_gps_novatel_ && (checker_b_map_data_init_ || param_b_localization_type_gps_)){
        gps_init_ublox_data.timestamp     = ros::Time::now().sec * 1e6 + ros::Time::now().nsec / 1e3;
        gps_init_ublox_data.sec           = ros::Time::now().sec;
        gps_init_ublox_data.nsec          = ros::Time::now().nsec;
        gps_init_ublox_data.latitude    = msg->lat/10000000.0;
        gps_init_ublox_data.longitude   = msg->lon/10000000.0;
        gps_init_ublox_data.height      = msg->height/1000.0;
        gps_init_ublox_data.yaw         = (-1. * msg->headVeh / 100000.0 + d_ublox_heading_bias_deg) * M_PI /180.;
        gps_init_ublox_data.vel_ms      = msg->speed/1000.0;
        gps_init_ublox_data.latitude_std  = msg->hAcc/1000.0;
        gps_init_ublox_data.longitude_std = msg->vAcc/1000.0;

        psstp_init_ublox_enu_pose_ = ConvertToMapFrame(gps_init_ublox_data.latitude, gps_init_ublox_data.longitude, gps_init_ublox_data.height);
        array_ego_vehicle_pose_[0] = psstp_init_ublox_enu_pose_.pose.position.x;
        array_ego_vehicle_pose_[1] = psstp_init_ublox_enu_pose_.pose.position.y;
        array_ego_vehicle_pose_[2] = gps_init_ublox_data.yaw;

        PFParticleInit(array_ego_vehicle_pose_);

        checker_b_gnss_data_init_ = true;
    }

    checker_b_ublox_data_exist_ = true;
    
    gps_ublox_data_.timestamp = ros::Time::now().sec * 1e6 + ros::Time::now().nsec / 1e3;

    std::string str_gps_quality;

    switch(msg->gpsFix)
    {
        case 0:
            gps_ublox_data_.quality = 6;
            str_gps_quality = "NO FIX";
            break;
        case 1:
            gps_ublox_data_.quality = 4;
            str_gps_quality = "DEAD RECKONING ONLY";
            break;
        case 2:
            gps_ublox_data_.quality = 3;
            str_gps_quality = "2D";
            break;
        case 3:
            gps_ublox_data_.quality = 2;
            str_gps_quality = "3D";
            break;
        case 4:
            gps_ublox_data_.quality = 1;
            str_gps_quality = "GPS DEAD RECKONING COMBINED";
            break;
        case 5:
            gps_ublox_data_.quality = 5;
            str_gps_quality = "TIME ONLY";
            break;
        default:
            gps_ublox_data_.quality = -1;
            str_gps_quality = "INVALID QUALITY TYPE";
            break;
    }

    if(!param_b_refernce_gps_novatel_)
    {
        std_msgs::String strmsg_gps_quality;
        strmsg_gps_quality.data = "Ublox GPS Quality: " + str_gps_quality;
        rospub_gps_quality_.publish(strmsg_gps_quality);
    }

    gps_ublox_data_.latitude  = msg->lat/10000000.0;
    gps_ublox_data_.longitude = msg->lon/10000000.0;
    gps_ublox_data_.height    = msg->height/1000.0;
    gps_ublox_data_.latitude_std  = msg->hAcc/1000.0;
    gps_ublox_data_.longitude_std = msg->vAcc/1000.0;

    gps_ublox_data_.yaw       = (-1. * msg->headVeh / 100000.0 + d_ublox_heading_bias_deg) * M_PI /180.;
    gps_ublox_data_.vel_ms    = msg->speed/1000.0;
    
    geometry_msgs::PoseStamped psstp_gnss_enu_pose_;

    psstp_gnss_enu_pose_ = ConvertToMapFrame(gps_ublox_data_.latitude, gps_ublox_data_.longitude, gps_ublox_data_.height);
    double array_tmp_gps_state[3];
    array_tmp_gps_state[0] = psstp_gnss_enu_pose_.pose.position.x;
    array_tmp_gps_state[1] = psstp_gnss_enu_pose_.pose.position.y;
    array_tmp_gps_state[2] = gps_ublox_data_.yaw;
        
    if(!param_b_refernce_gps_novatel_)
    {
        if (PFFaultDetection())
        {
            PFParticleInit(array_tmp_gps_state);
            ROS_ERROR_STREAM("Particle Filter Fault Detection !!! ");
        }
    }
}

// 0. FAULT DETECTION & PARTICLE INITIALIZATION (RESET)
bool LandmarkMatchingPF::PFParticleInit(double array_state[])
{
    Eigen::MatrixXd egmat_init_state(param_i_num_particle,i_num_of_state_);
    for(int idx_particle = 0; idx_particle<param_i_num_particle;idx_particle++)
    {
        egmat_init_state(idx_particle, 0) = array_state[0];
        egmat_init_state(idx_particle, 1) = array_state[1];
        egmat_init_state(idx_particle, 2) = array_state[2];

        (*egmat_particle_weight_)(idx_particle, 0) = (1. / (double)param_i_num_particle);
    }
    *egmat_timestamped_particle_state_ = egmat_init_state;
}

bool LandmarkMatchingPF::PFFaultDetection(void)
{   
    double d_gps_positioin_quality_threshold_m;
    double d_gps_fault_position_threshold_m;
    double d_gps_fault_heading_threshold_deg;

    d_gps_positioin_quality_threshold_m = param_d_gnss_position_quality_threshold_m_;
    d_gps_fault_position_threshold_m   = param_d_gnss_fault_position_threshold_m_;
    d_gps_fault_heading_threshold_deg  = param_d_gnss_fault_heading_threshold_deg_;

    GpsData gps_gps;

    if(param_b_refernce_gps_novatel_)
    {
        gps_gps = gps_gps_data_;
    }
    else
    {
        gps_gps = gps_ublox_data_;
    }

    if((int)gps_gps.quality>3)
    {
        ROS_WARN_STREAM("GPS QUALITY IS TOO LOW!");
        return false;
    }
    
    double d_gps_pos_error = sqrt(
          (double)gps_gps.latitude_std *gps_gps.latitude_std
        + (double)gps_gps.longitude_std*gps_gps.longitude_std);

	if (d_gps_pos_error > d_gps_positioin_quality_threshold_m)
	{
		return false;
	}

    geometry_msgs::PoseStamped psstp_tmp_gps_enu;
    psstp_tmp_gps_enu = ConvertToMapFrame(gps_gps.latitude, gps_gps.longitude, gps_gps.height);

    double d_tmp_gps_east_m        = psstp_tmp_gps_enu.pose.position.x;
    double d_tmp_gps_north_m       = psstp_tmp_gps_enu.pose.position.y;
    double d_tmp_gps_heading_rad   = gps_gps.yaw;

    double d_tmp_PF_east_m          = egmat_estimated_state_->coeff(0,0);
    double d_tmp_PF_north_m         = egmat_estimated_state_->coeff(0,1);
    double d_tmp_PF_heading_rad     = egmat_estimated_state_->coeff(0,2);

    double d_distance_between_PF_gps = sqrt(pow(abs(d_tmp_gps_east_m  - d_tmp_PF_east_m) ,2)
                                        +pow(abs(d_tmp_gps_north_m - d_tmp_PF_north_m),2));

	// Heading Distance between geo-position between PF and GPS
	double d_heading_diff_between_PF_gps = abs(d_tmp_gps_heading_rad - d_tmp_PF_heading_rad) * 180.0 / M_PI;

	// Checking threshold for difference between PF and GPS
	if (d_distance_between_PF_gps    <= d_gps_fault_position_threshold_m &&
		d_heading_diff_between_PF_gps <= d_gps_fault_heading_threshold_deg)
	{
		return false;
	}

	return true;
}

// 1. PREDICTION 
bool LandmarkMatchingPF::PFPrediction(void)
{
    double array_tmp_input[3] = {0. , 0. , 0.};
    double array_tmp_state[3] = {0. , 0. , 0.};
    Eigen::MatrixXd egmat_pred_state(param_i_num_particle,i_num_of_state_);

    static double d_prev_timestamp_ms = imu_imu_data_.timestamp;

    double d_delta_t = (imu_imu_data_.timestamp - d_prev_timestamp_ms)/1e6;
    d_prev_timestamp_ms = imu_imu_data_.timestamp;

    for(int idx_particle = 0; idx_particle < param_i_num_particle; idx_particle++)
    {
        array_tmp_input[0] = imu_imu_data_.vel_ms           + GaussianRandomGenerator(0, param_d_input_vel_sigma_ms_);
        array_tmp_input[1] = imu_imu_data_.yaw_rate_rads    + GaussianRandomGenerator(0, param_d_input_yaw_rate_sigma_rads_);
        array_tmp_input[2] = imu_imu_data_.acc_mss          + GaussianRandomGenerator(0, param_d_input_acc_sigma_mss_);

        array_tmp_state[0] = egmat_timestamped_particle_state_->coeff(idx_particle, 0) + GaussianRandomGenerator(0, param_d_east_sigma_m_);
        array_tmp_state[1] = egmat_timestamped_particle_state_->coeff(idx_particle, 1) + GaussianRandomGenerator(0, param_d_north_sigma_m_);
        array_tmp_state[2] = egmat_timestamped_particle_state_->coeff(idx_particle, 2) + GaussianRandomGenerator(0, param_d_heading_sigma_rad_);

        // Prediction with CV Model
        if(d_delta_t > 1.0){
            return false;
        }

        array_tmp_state[0]  = array_tmp_state[0]  + array_tmp_input[0] * d_delta_t * cos(array_tmp_state[2]);
        array_tmp_state[1] = array_tmp_state[1] + array_tmp_input[0] * d_delta_t * sin(array_tmp_state[2]);
        array_tmp_state[2] = array_tmp_state[2] + d_delta_t * array_tmp_input[1];
        
        checker_b_imu_data_exist_ = false;

        egmat_pred_state(idx_particle, 0) = array_tmp_state[0];
        egmat_pred_state(idx_particle, 1) = array_tmp_state[1];
        egmat_pred_state(idx_particle, 2) = array_tmp_state[2];
    }
    *egmat_timestamped_particle_state_ = egmat_pred_state;
}

// 2. REPRESENTATIVE POSE EXTRACTION
bool LandmarkMatchingPF::PFRepresentativePoseExtraction(void)
{
    double d_weight_mean_east_m        = 0.;
    double d_weight_mean_north_m       = 0.;
    double d_weight_mean_heading_rad   = 0.;

    double d_weight_max_east_m         = 0.;
    double d_weight_max_north_m        = 0.;
    double d_weight_max_heading_rad    = 0.;
    double d_max_weight               = 0.;

    double d_weight_variance_east_m = 0.;
    double d_weight_variance_north_m = 0.;
    double d_weight_variance_heading_rad = 0.;

    double d_weight_square_sum = 0.;

    for (int idx_particle = 0; idx_particle < param_i_num_particle; idx_particle++)
    {
        d_weight_square_sum += egmat_particle_weight_->coeff(idx_particle,0) * egmat_particle_weight_->coeff(idx_particle,0);
    }

    if(d_weight_square_sum <= 0.)
    {
        d_weight_variance_east_m = 0.;
        d_weight_variance_north_m = 0.;
        d_weight_variance_heading_rad = 0.;
        return false;
    }

    for(int idx_particle = 0; idx_particle < param_i_num_particle; idx_particle++)
    {
        if(egmat_particle_weight_->coeff(idx_particle,0) > d_max_weight)
        {
            d_weight_max_east_m      = egmat_timestamped_particle_state_->coeff(idx_particle, 0);
            d_weight_max_north_m     = egmat_timestamped_particle_state_->coeff(idx_particle, 1);
            d_weight_max_heading_rad = egmat_timestamped_particle_state_->coeff(idx_particle, 2);
            d_max_weight            = egmat_particle_weight_->coeff(idx_particle,0);
        }

        d_weight_mean_east_m      += egmat_particle_weight_->coeff(idx_particle,0) * egmat_timestamped_particle_state_->coeff(idx_particle, 0);
        d_weight_mean_north_m     += egmat_particle_weight_->coeff(idx_particle,0) * egmat_timestamped_particle_state_->coeff(idx_particle, 1);
        // sin,cos vs d_reference_heading_rad
        d_weight_mean_heading_rad += egmat_particle_weight_->coeff(idx_particle,0) * egmat_timestamped_particle_state_->coeff(idx_particle, 2);
    }

    for(int idx_particle = 0; idx_particle < param_i_num_particle; idx_particle++)
    {
        double d_diff_east_m      = egmat_timestamped_particle_state_->coeff(idx_particle, 0) - d_weight_mean_east_m;
        double d_diff_north_m     = egmat_timestamped_particle_state_->coeff(idx_particle, 1) - d_weight_mean_north_m;
        double d_diff_heading_rad = egmat_timestamped_particle_state_->coeff(idx_particle, 2) - d_weight_mean_heading_rad;
        if(d_diff_east_m > 100)
            std::cout<<"out of lange particle idx is"<<idx_particle<<std::endl;
        d_weight_variance_east_m      += egmat_particle_weight_->coeff(idx_particle,0) * d_diff_east_m * d_diff_east_m;
        d_weight_variance_north_m     += egmat_particle_weight_->coeff(idx_particle,0) * d_diff_north_m * d_diff_north_m;
        d_weight_variance_heading_rad += egmat_particle_weight_->coeff(idx_particle,0) * d_diff_heading_rad * d_diff_heading_rad;
    }

    double d_one_minus_weight_square_sum = 1. - d_weight_square_sum;

    if(d_one_minus_weight_square_sum <= FLT_MIN)
    {
        d_weight_variance_east_m = 0.;
        d_weight_variance_north_m = 0.;
        d_weight_variance_heading_rad = 0.;
        return false;
    }
    else
    {
        d_weight_variance_east_m      /= d_one_minus_weight_square_sum;
        d_weight_variance_north_m     /= d_one_minus_weight_square_sum;
        d_weight_variance_heading_rad /= d_one_minus_weight_square_sum;
    }

    (*egmat_estimated_state_)(0,0) = d_weight_mean_east_m;
    (*egmat_estimated_state_)(0,1) = d_weight_mean_north_m;
    (*egmat_estimated_state_)(0,2) = d_weight_mean_heading_rad;

    (*egmat_estimated_sigma_)(0,0) = sqrt(d_weight_variance_east_m);
    (*egmat_estimated_sigma_)(0,1) = sqrt(d_weight_variance_north_m);
    (*egmat_estimated_sigma_)(0,2) = sqrt(d_weight_variance_heading_rad);

    return true;
}

// 3. RESAMPLING
bool LandmarkMatchingPF::PFResampling(void)
{
    // Compute an estimate of the effective number of particle
	double d_sqaure_sum_weight = 0.;
    
	for (int idx_particle = 0; idx_particle < param_i_num_particle; idx_particle++)
    {
		d_sqaure_sum_weight += egmat_particle_weight_->coeff(idx_particle,0) * egmat_particle_weight_->coeff(idx_particle,0); // Square sum of weight
    }

	double d_eff_particle_num;
	if (d_sqaure_sum_weight > FLT_MIN)
	{
		d_eff_particle_num = 1. / d_sqaure_sum_weight;
	}
	else
	{
		return false;
	}

    //Resampling
    *egmat_timestamped_particle_state_resampling_ = *egmat_timestamped_particle_state_;

    if(d_eff_particle_num < param_d_resampling_coefficent_ * param_i_num_particle)
    {
        // Low variance re-sampling		
		double d_init_random_num = GaussianRandomGenerator(0., 1. / param_i_num_particle);
		double d_accumlate_weight = egmat_particle_weight_->coeff(0, 0);
		int	   i_particle_resample = 0;	// re-sample particle index

		for (int i_new_particle_num = 0; i_new_particle_num < param_i_num_particle; i_new_particle_num++)
		{
			// Find particle for resampling
			double d_weight_interval = /*d_init_random_num +*/ (double)(i_new_particle_num) / param_i_num_particle;
			while (d_weight_interval > d_accumlate_weight)
			{
                i_particle_resample++;
				d_accumlate_weight += egmat_particle_weight_->coeff(i_particle_resample,0);
			}
            if(i_particle_resample > param_i_num_particle-1)
            {
                i_particle_resample = param_i_num_particle - 1;
            }

            for (int i_state_idx = 0; i_state_idx < i_num_of_state_; i_state_idx++)
            {
                (*egmat_timestamped_particle_state_resampling_)(i_new_particle_num, i_state_idx) =
                    egmat_timestamped_particle_state_->coeff(i_particle_resample, i_state_idx);
            }

		}

        // Weight initialization 
		for (int idx_particle = 0; idx_particle < param_i_num_particle; idx_particle++)
		{
			// Weight initialization 
			(*egmat_particle_weight_)(idx_particle,0) = 1. / (double)param_i_num_particle;
		}

        Eigen::MatrixXd* egmat_swap_pointer = egmat_timestamped_particle_state_;
		egmat_timestamped_particle_state_ = egmat_timestamped_particle_state_resampling_;
		egmat_timestamped_particle_state_resampling_ = egmat_swap_pointer;
		*egmat_timestamped_particle_state_resampling_ = Eigen::MatrixXd(param_i_num_particle, i_num_of_state_); //reset

		return true;
    }
    else
    {
        return false;
    }
}

// 4. MEASUREMENT UPDATE
void LandmarkMatchingPF::WeightUpdate(Eigen::MatrixXd* egmat_input_weight_mat)
{
    // PF weight update part
    for(int i_weight_idx = 0; i_weight_idx<param_i_num_particle; i_weight_idx++)
    {
        (*egmat_particle_weight_)(i_weight_idx,0) = egmat_particle_weight_->coeff(i_weight_idx,0) * egmat_input_weight_mat->coeff(i_weight_idx,0);
    }

    if(!NormalizationWeight(egmat_particle_weight_))
    {
        ROS_ERROR_STREAM("Weight normalization error !!!!");
    }
}

bool LandmarkMatchingPF::NormalizationWeight(Eigen::MatrixXd* egmat_input_matrix)
{
    double d_weight_num = param_i_num_particle;
    double d_weight_sum = 0.;
    for(int i_mat_idx = 0; i_mat_idx < d_weight_num;i_mat_idx++)
    {
        d_weight_sum += abs(egmat_input_matrix->coeff(i_mat_idx, 0));
    }
    if(d_weight_sum> 0.)
    {
        for(int i_mat_idx = 0; i_mat_idx<d_weight_num;i_mat_idx++)
        {
            (*egmat_input_matrix)(i_mat_idx, 0) = egmat_input_matrix->coeff(i_mat_idx, 0)/d_weight_sum;
        }
        return true;
    }
    else
    {
        return false;
    }
}

// 4-1. GPS BASED MEASUREMENT UPDATE
void LandmarkMatchingPF::GPSMeasurementUpdate(void)
{
    geometry_msgs::PoseStamped psstp_tmp_gps_enu;
    GpsData gps_gps;

    if(param_b_refernce_gps_novatel_){
        gps_gps = gps_gps_data_;
    }
    else
    {
        gps_gps = gps_ublox_data_;
    }

    psstp_tmp_gps_enu = ConvertToMapFrame(gps_gps.latitude, gps_gps.longitude, gps_gps.height);

    double d_tmp_east_m = psstp_tmp_gps_enu.pose.position.x;
    double d_tmp_north_m = psstp_tmp_gps_enu.pose.position.y;

    for(int idx_particle = 0; idx_particle < param_i_num_particle; idx_particle++)
    {
        double d_particle_east_m = egmat_timestamped_particle_state_->coeff(idx_particle, 0);
        double d_particle_north_m = egmat_timestamped_particle_state_->coeff(idx_particle, 1);

        double d_tmp_diff_distance_m = sqrt(pow((d_tmp_east_m - d_particle_east_m),2)+pow((d_tmp_north_m - d_particle_north_m),2));
        if(d_tmp_diff_distance_m <= param_d_gnss_radius_boundary_m_)
        {
            (*egmat_gps_particle_likelihood_)(idx_particle, 0) = 1.;
        }
        else
        {
            (*egmat_gps_particle_likelihood_)(idx_particle, 0) = 0.;
        }
    }
    if(!NormalizationWeight(egmat_gps_particle_likelihood_))
    {
        ROS_ERROR_STREAM("Weight normalization error !!!!");
    }
    else
    {
        WeightUpdate(egmat_gps_particle_likelihood_);
    }
    
}

// 4-2. POINT CLOUD MAP MATCHING BASED UPDATE
void LandmarkMatchingPF::MapMatchingMeasurementUpdate(void)
{
    pcl::PointXYZRGB pt_search_point;
    pt_search_point.x = egmat_estimated_state_->coeff(0,0);
    pt_search_point.y = egmat_estimated_state_->coeff(0,1);
    pt_search_point.z = d_map_height_;

    // std::vector<int> pointIdxRadiusSearch;
    std::vector<int> vec_point_idx_random_sample;
    std::vector<float> vec_point_radius_squred_distance;

    std::vector<int> vec_point_idx_random_num;
    vec_point_idx_random_num.clear();

    unsigned int ui_input_point_cloud_size = (unsigned int)pcptr_input_point_cloud_->points.size();
    unsigned int ui_random_selected_idx;
 
    for (int i_sample_point_num = 0; i_sample_point_num < param_i_num_map_matching_random_sample_point_; i_sample_point_num++)
    {
        ui_random_selected_idx = (unsigned int)rand() % ui_input_point_cloud_size;
        vec_point_idx_random_num.push_back(ui_random_selected_idx);
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcptr_transformed_point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    for(int idx_particle = 0; idx_particle < param_i_num_particle; idx_particle++)
    {
        pcptr_transformed_point_cloud->clear();

        double d_particle_east_m = egmat_timestamped_particle_state_->coeff(idx_particle, 0);
        double d_particle_north_m = egmat_timestamped_particle_state_->coeff(idx_particle, 1);
        double d_particle_heading_rad = egmat_timestamped_particle_state_->coeff(idx_particle, 2);

        tf::Quaternion tfquat_quterinon;
        tfquat_quterinon.setRPY(0., 0., d_particle_heading_rad);

        // Get tfvec_offset
        tf::Vector3 tfvec_offset = tf::Vector3(d_particle_east_m, d_particle_north_m, d_map_height_);

        tf::Transform tf_transformation = tf::Transform(tfquat_quterinon, tfvec_offset);
        
        pcl_ros::transformPointCloud(*pcptr_input_point_cloud_, *pcptr_transformed_point_cloud, tf_transformation);

        for(int i_point_idx = 0; i_point_idx < param_i_num_map_matching_random_sample_point_; i_point_idx++)
        {
            pcl::PointXYZRGB pt_search_point;
            pt_search_point.x = pcptr_transformed_point_cloud->points[vec_point_idx_random_num[i_point_idx]].x;
            pt_search_point.y = pcptr_transformed_point_cloud->points[vec_point_idx_random_num[i_point_idx]].y;
            pt_search_point.z = pcptr_transformed_point_cloud->points[vec_point_idx_random_num[i_point_idx]].z;
            
			// Scoring using distance functions
			double d_likelihood = 0.0;
            std::vector<int> vec_point_idx_random_sample;
            std::vector<float> vec_point_radius_squred_distance;

			kdtree_global_map_->nearestKSearch(pt_search_point, 1, vec_point_idx_random_sample, vec_point_radius_squred_distance);
            double d_min_dist = vec_point_radius_squred_distance[0];

            if (d_min_dist < param_d_map_sigma_*param_d_map_noise_measure_noise_sigma_m_ || param_d_map_sigma_<0.) 
            {
				d_likelihood = exp(-d_min_dist*d_min_dist / (param_d_map_noise_measure_noise_sigma_m_*param_d_map_noise_measure_noise_sigma_m_) / 2);
			}
			
            (*egmat_map_matching_particle_likelihood_)(idx_particle,0) += d_likelihood;
        }
    }
    SaveResultText(egmat_map_matching_particle_likelihood_, "MapMatching");

    if(NormalizationWeight(egmat_map_matching_particle_likelihood_))
    {
        WeightUpdate(egmat_map_matching_particle_likelihood_);
    }
}

// 4-3. IMAGE & SPC MATCHING BASED UPDATE
void LandmarkMatchingPF::ImageSPCMeasurementUpdate(void)
{
    for(int idx_particle = 0; idx_particle < param_i_num_particle; idx_particle++)
    {
        double d_likelihood_pole          = ImageContourSPCMatching(vec_image_contour_point_pole_, vec_particle_based_projected_points_pole_[idx_particle]);
        double d_likelihood_traffic_sign   = ImageContourSPCMatching(vec_image_contour_point_traffic_sign_, vec_particle_based_projected_points_traffic_sign_[idx_particle]);
        double d_likelihood_traffic_light  = ImageContourSPCMatching(vec_image_contour_point_traffic_light_, vec_particle_based_projected_points_traffic_light_[idx_particle]);
        double d_likelihood_tunnel_fan     = ImageContourSPCMatching(vec_image_contour_point_tunnel_fan_, vec_particle_based_projected_points_tunnel_fan_[idx_particle]);
        double dLikelihoodTunnelLight   = ImageContourSPCMatching(vec_image_contour_point_tunnel_light_, vec_particle_based_projected_points_tunnel_light_[idx_particle]);
        double d_likelihood_tunnel_hydrant = ImageContourSPCMatching(vec_image_contour_point_tunnel_hydrant, vec_particle_based_projected_points_tunnel_hydrant_[idx_particle]);
        
        double d_likelihood_building      = SegmentedImageSPCMatching(vec_particle_based_projected_points_building_[idx_particle],    cvvec_lane_semantic_rgb_, 1);
        double d_likelihood_lane_marking   = SegmentedImageSPCMatching(vec_particle_based_projected_points_lane_marking_[idx_particle], cvvec_lane_semantic_rgb_, 15);
        double d_likelihood_fence    = SegmentedImageSPCMatching(vec_particle_based_projected_points_fence_[idx_particle],      cvvec_fence_semantic_rgb_, 2);

        (*egmat_pole_particle_likelihood_)(idx_particle,0)          = d_likelihood_pole;
        (*egmat_traffic_sign_particle_likelihood_)(idx_particle,0)   = d_likelihood_traffic_sign;
        (*egmat_traffic_light_particle_likelihood_)(idx_particle,0)  = d_likelihood_traffic_light;
        (*egmat_tunnel_fan_particle_likeilhood_)(idx_particle,0)     = d_likelihood_tunnel_fan;
        (*egmat_tunnel_light_particle_likelihood_)(idx_particle,0)   = dLikelihoodTunnelLight;
        (*egmat_tunnel_hydrant_particle_likelihood_)(idx_particle,0) = d_likelihood_tunnel_hydrant;

        (*egmat_building_particle_likelihood_)(idx_particle,0)      = d_likelihood_building;
        (*egmat_lane_marking_particle_likelihood_)(idx_particle,0)   = d_likelihood_lane_marking;
        (*egmat_fence_particle_likelihood_)(idx_particle,0)         = d_likelihood_fence;
    }

    std_msgs::String stdstr_score_check_information;

    if(egmat_pole_particle_likelihood_->maxCoeff()>param_d_pole_likelihood_thresholde_)
    {
        stdstr_score_check_information.data += "POLE UPDATE : "+std::to_string(egmat_pole_particle_likelihood_->maxCoeff())+"\n";

        if(egmat_pole_particle_likelihood_->minCoeff()>0.)
        {
            Eigen::MatrixXd egmat_min_subtracted_likelihood(param_i_num_particle, 1);
            egmat_min_subtracted_likelihood = egmat_pole_particle_likelihood_->array() - egmat_pole_particle_likelihood_->minCoeff();

            *egmat_pole_particle_likelihood_ = egmat_min_subtracted_likelihood;
        }

        if(NormalizationWeight(egmat_pole_particle_likelihood_))
        {
            WeightUpdate(egmat_pole_particle_likelihood_);
        }
    }
    else
    {
        stdstr_score_check_information.data += "POLE NOT UPDATE : "+std::to_string(egmat_pole_particle_likelihood_->maxCoeff())+"\n";
    }

    if(egmat_traffic_sign_particle_likelihood_->maxCoeff()>param_d_traffic_sign_likelihood_thresholde_)
    {
        stdstr_score_check_information.data += "TRAFFIC SIGN UPDATE : "+std::to_string(egmat_traffic_sign_particle_likelihood_->maxCoeff())+"\n";

        if(egmat_traffic_sign_particle_likelihood_->minCoeff()>0.)
        {
            Eigen::MatrixXd egmat_min_subtracted_likelihood(param_i_num_particle, 1);
            egmat_min_subtracted_likelihood = egmat_traffic_sign_particle_likelihood_->array() - egmat_traffic_sign_particle_likelihood_->minCoeff();

            *egmat_traffic_sign_particle_likelihood_ = egmat_min_subtracted_likelihood;
        }

        if(NormalizationWeight(egmat_traffic_sign_particle_likelihood_))
        {
            WeightUpdate(egmat_traffic_sign_particle_likelihood_);
        }
    }
    else
    {
        stdstr_score_check_information.data += "TRAFFIC SIGN NOT UPDATE : "+std::to_string(egmat_traffic_sign_particle_likelihood_->maxCoeff())+"\n";
    }

    if(egmat_traffic_light_particle_likelihood_->maxCoeff()>param_d_traffic_light_likelihood_threshold_)
    {
        stdstr_score_check_information.data += "TRAFFIC LIGHT UPDATE : "+std::to_string(egmat_traffic_light_particle_likelihood_->maxCoeff())+"\n";

        if(egmat_traffic_light_particle_likelihood_->minCoeff()>0.)
        {
            Eigen::MatrixXd egmat_min_subtracted_likelihood(param_i_num_particle, 1);
            egmat_min_subtracted_likelihood = egmat_traffic_light_particle_likelihood_->array() - egmat_traffic_light_particle_likelihood_->minCoeff();

            *egmat_traffic_light_particle_likelihood_ = egmat_min_subtracted_likelihood;
        }

        if(NormalizationWeight(egmat_traffic_light_particle_likelihood_))
        {
            WeightUpdate(egmat_traffic_light_particle_likelihood_);
        }
    }
    else
    {
        stdstr_score_check_information.data += "TRAFFIC LIGHT NOT UPDATE : "+std::to_string(egmat_traffic_light_particle_likelihood_->maxCoeff())+"\n";
    }

    if(egmat_tunnel_fan_particle_likeilhood_->maxCoeff()>param_d_tunnel_fan_likelihood_threshold_)
    {
        stdstr_score_check_information.data += "TUNNEL FAN UPDATE : "+std::to_string(egmat_tunnel_fan_particle_likeilhood_->maxCoeff())+"\n";

        if(egmat_tunnel_fan_particle_likeilhood_->minCoeff()>0.)
        {
            Eigen::MatrixXd egmat_min_subtracted_likelihood(param_i_num_particle, 1);
            egmat_min_subtracted_likelihood = egmat_tunnel_fan_particle_likeilhood_->array() - egmat_tunnel_fan_particle_likeilhood_->minCoeff();

            *egmat_tunnel_fan_particle_likeilhood_ = egmat_min_subtracted_likelihood;
        }

        if(NormalizationWeight(egmat_tunnel_fan_particle_likeilhood_))
        {
            WeightUpdate(egmat_tunnel_fan_particle_likeilhood_);
        }
    }
    else
    {
        stdstr_score_check_information.data += "TUNNEL FAN NOT UPDATE : "+std::to_string(egmat_tunnel_fan_particle_likeilhood_->maxCoeff())+"\n";
    }

    if(egmat_tunnel_light_particle_likelihood_->maxCoeff()>param_d_tunnel_light_likelihood_threshold_)
    {
        stdstr_score_check_information.data += "TUNNEL LIGHT UPDATE : "+std::to_string(egmat_tunnel_light_particle_likelihood_->maxCoeff())+"\n";

        if(egmat_tunnel_light_particle_likelihood_->minCoeff()>0.)
        {
            Eigen::MatrixXd egmat_min_subtracted_likelihood(param_i_num_particle, 1);
            egmat_min_subtracted_likelihood = egmat_tunnel_light_particle_likelihood_->array() - egmat_tunnel_light_particle_likelihood_->minCoeff();

            *egmat_tunnel_light_particle_likelihood_ = egmat_min_subtracted_likelihood;
        }

        if(NormalizationWeight(egmat_tunnel_light_particle_likelihood_))
        {
            WeightUpdate(egmat_tunnel_light_particle_likelihood_);
        }
    }
    else
    {
        stdstr_score_check_information.data += "TUNNEL LIGHT NOT UPDATE : "+std::to_string(egmat_tunnel_light_particle_likelihood_->maxCoeff())+"\n";
    }

    if(egmat_tunnel_hydrant_particle_likelihood_->maxCoeff()>param_d_tunnel_hydrant_likelihood_threshold_)
    {
        stdstr_score_check_information.data += "TUNNEL HYDRANT UPDATE : "+std::to_string(egmat_tunnel_hydrant_particle_likelihood_->maxCoeff())+"\n";

        if(egmat_tunnel_hydrant_particle_likelihood_->minCoeff()>0.)
        {
            Eigen::MatrixXd egmat_min_subtracted_likelihood(param_i_num_particle, 1);
            egmat_min_subtracted_likelihood = egmat_tunnel_hydrant_particle_likelihood_->array() - egmat_tunnel_hydrant_particle_likelihood_->minCoeff();

            *egmat_tunnel_hydrant_particle_likelihood_ = egmat_min_subtracted_likelihood;
        }

        if(NormalizationWeight(egmat_tunnel_hydrant_particle_likelihood_))
        {
            WeightUpdate(egmat_tunnel_hydrant_particle_likelihood_);
        }
    }
    else
    {
        stdstr_score_check_information.data += "TUNNEL HYDRANT NOT UPDATE : "+std::to_string(egmat_tunnel_hydrant_particle_likelihood_->maxCoeff())+"\n";
    }

    if(egmat_building_particle_likelihood_->maxCoeff()>param_d_building_likelihood_threshold_)
    {
        stdstr_score_check_information.data += "BUILDING UPDATE : "+std::to_string(egmat_building_particle_likelihood_->maxCoeff())+"\n";

        if(egmat_building_particle_likelihood_->minCoeff()>0.)
        {
            Eigen::MatrixXd egmat_min_subtracted_likelihood(param_i_num_particle, 1);
            egmat_min_subtracted_likelihood = egmat_building_particle_likelihood_->array() - egmat_building_particle_likelihood_->minCoeff();

            *egmat_building_particle_likelihood_ = egmat_min_subtracted_likelihood;
        }

        if(NormalizationWeight(egmat_building_particle_likelihood_))
        {
            WeightUpdate(egmat_building_particle_likelihood_);
        }
    }
    else
    {
        stdstr_score_check_information.data += "BUILDING NOT UPDATE : "+std::to_string(egmat_building_particle_likelihood_->maxCoeff())+"\n";
    }

    if(egmat_lane_marking_particle_likelihood_->maxCoeff()>param_d_lane_marking_likelihood_threshold_)
    {
        stdstr_score_check_information.data += "LANE MARKING UPDATE : "+std::to_string(egmat_lane_marking_particle_likelihood_->maxCoeff())+"\n";

        if(egmat_lane_marking_particle_likelihood_->minCoeff()>0.)
        {
            Eigen::MatrixXd egmat_min_subtracted_likelihood(param_i_num_particle, 1);
            egmat_min_subtracted_likelihood = egmat_lane_marking_particle_likelihood_->array() - egmat_lane_marking_particle_likelihood_->minCoeff();

            *egmat_lane_marking_particle_likelihood_ = egmat_min_subtracted_likelihood;
        }
        SaveResultText(egmat_lane_marking_particle_likelihood_, "Lane Matching");

        if(NormalizationWeight(egmat_lane_marking_particle_likelihood_))
        {
            WeightUpdate(egmat_lane_marking_particle_likelihood_);
        }
    }
    else
    {
        stdstr_score_check_information.data += "LANE MARKING NOT UPDATE : "+std::to_string(egmat_lane_marking_particle_likelihood_->maxCoeff())+"\n";
    }
    

    if(egmat_fence_particle_likelihood_->maxCoeff()>param_d_fence_likelihood_threshold_)
    {
        stdstr_score_check_information.data += "FENCE MARKING UPDATE : "+std::to_string(egmat_fence_particle_likelihood_->maxCoeff());

        if(egmat_fence_particle_likelihood_->minCoeff()>0.)
        {
            Eigen::MatrixXd egmat_min_subtracted_likelihood(param_i_num_particle, 1);
            egmat_min_subtracted_likelihood = egmat_fence_particle_likelihood_->array() - egmat_fence_particle_likelihood_->minCoeff();

            *egmat_fence_particle_likelihood_ = egmat_min_subtracted_likelihood;
        }

        if(NormalizationWeight(egmat_fence_particle_likelihood_))
        {
            WeightUpdate(egmat_fence_particle_likelihood_);
        }
    }
    else
    {
        stdstr_score_check_information.data += "FENCE NOT UPDATE : "+std::to_string(egmat_fence_particle_likelihood_->maxCoeff());
    }

    rospub_matching_score_.publish(stdstr_score_check_information);
}

double LandmarkMatchingPF::ImageContourSPCMatching(std::vector<std::vector<cv::Point>> vec_input_contours, std::vector<cv::Point2d> vec_input_points)
{
    // Intersection = (# of intersection points / contour area)*
    if(vec_input_contours.size()==0 || vec_input_points.size() == 0)
    {
        return 0.;
    }

    int i_intersection_points_count = 0;

    for(auto& contour : vec_input_contours)
    {
        for(auto& point : vec_input_points)
        {
            if(CheckPointsInsideContour(contour, point)>=0.)
            {
                i_intersection_points_count++;
            }
        }
    }
    
    double d_intersection_score = (double)i_intersection_points_count;
    return d_intersection_score;
}

double LandmarkMatchingPF::SegmentedImageSPCMatching(std::vector<cv::Point2d> vec_input_points, cv::Vec3b cvvec_ref_bgr, int i_pixel_bias)
{
    if(vec_input_points.size() == 0 || cvmat_segmented_image_.empty())
    {
        return 0.;
    }

    double i_intersection_points_count = 0.;
    
    for(auto& point : vec_input_points)
    {
        if((int)point.x < cvmat_segmented_image_.cols && (int)point.x > 0.
            && (int)point.y < cvmat_segmented_image_.rows && (int)point.y > 0.)
        {
            for(auto pixel_idx = 0; pixel_idx < i_pixel_bias; pixel_idx++)
            {
                if(CheckNearbyIntersectionPointExist(point, cvvec_ref_bgr, pixel_idx))
                {
                    i_intersection_points_count =  i_intersection_points_count + (1.0 - pixel_idx*0.05);
                    // i_intersection_points_count =  i_intersection_points_count + (1.0/(pixel_idx));
                    break;
                }
            }
        }
    }

    double d_intersection_score = (double)i_intersection_points_count;

    return d_intersection_score;
}

// 4-3-1. UTIL FUNCTIONS
void LandmarkMatchingPF::PointCloud2DProjection(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputPointCloud, Eigen::Matrix4d egmat_input_mat, std::vector<cv::Point2d>& vec_output_2d_projected_point)
{
    if(cvptr_input_image_ == NULL)
    {
        ROS_ERROR_STREAM("!!! IMAGE DATA DOES NOT EXIST !!!");
        return;
    }

    if(inputPointCloud->points.size()<1)
    {
        return;
    }

    Eigen::Matrix4d egmat_cam_to_lidar_transform_mat;
    Eigen::Matrix4d egmat_cam_to_particle_transform_mat;
    Eigen::Matrix4d egmat_lidar_to_particle_transform_mat = egmat_input_mat;

    double calibration_d_roll_deg  = param_d_extrinsic_calibration_roll_deg_* M_PI/180.;
    double calibration_d_pitch_deg = param_d_extrinsic_calibration_pitch_deg_ * M_PI/180.;
    double calibration_d_yaw_deg   = param_d_extrinsic_calibration_yaw_deg_ * M_PI/180.;

    double calibration_tx_m = param_d_extrinsic_calibration_x_m_;
    double calibratoin_ty_m = param_d_extrinsic_calibration_y_m_;
    double calibration_tz_m = param_d_extrinsic_calibration_z_m_;

    egmat_cam_to_lidar_transform_mat << cos(calibration_d_pitch_deg)*cos(calibration_d_yaw_deg), -cos(calibration_d_roll_deg)*sin(calibration_d_yaw_deg) + sin(calibration_d_roll_deg)*sin(calibration_d_pitch_deg)*cos(calibration_d_yaw_deg),  sin(calibration_d_roll_deg)*sin(calibration_d_yaw_deg)+cos(calibration_d_roll_deg)*sin(calibration_d_pitch_deg)*cos(calibration_d_yaw_deg), calibration_tx_m,
                                        cos(calibration_d_pitch_deg)*sin(calibration_d_yaw_deg),  cos(calibration_d_roll_deg)*cos(calibration_d_yaw_deg) + sin(calibration_d_roll_deg)*sin(calibration_d_pitch_deg)*sin(calibration_d_yaw_deg), -sin(calibration_d_roll_deg)*cos(calibration_d_yaw_deg)+cos(calibration_d_roll_deg)*sin(calibration_d_pitch_deg)*sin(calibration_d_yaw_deg), calibratoin_ty_m, 
                                        -sin(calibration_d_pitch_deg)         ,  sin(calibration_d_roll_deg)*cos(calibration_d_pitch_deg)                              ,  cos(calibration_d_roll_deg)*cos(calibration_d_pitch_deg)                            , calibration_tz_m, 
                                        0.                 , 0.                                                 , 0.                                               , 1.;

    egmat_cam_to_particle_transform_mat = egmat_cam_to_lidar_transform_mat*egmat_lidar_to_particle_transform_mat;

    cv::Mat cv_rotation_mat, cv_translate_vec;

    cv::Mat cv_camera_intrinsic_mat   (cv::Size(3,3), CV_64F, calibration_array_camera_intrinsic_matrix_);
    cv::Mat cv_camera_distortion_mat(cv::Size(1,5), CV_64F, calibration_array_camera_distortion_matrix_);
    cv::Mat cv_camera_extrinsic_mat;

    cv::eigen2cv(egmat_cam_to_particle_transform_mat,cv_camera_extrinsic_mat);

    cv::Mat R_zero          (cv::Size(3,3), CV_64F, calibration_array_zero_rotation_matrix_);
    cv::Mat t_zero          (cv::Size(3,1), CV_64F, calibration_array_zero_translation_vector_);
    cv::Mat cvmat_image_origin    = cvptr_input_image_->image;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcptr_rotated_point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    //read cv_rotation_mat t from Extrin_matrix
    cv_rotation_mat       = cv_camera_extrinsic_mat(cv::Range(0,3),cv::Range(0,3));
    cv_translate_vec   = cv_camera_extrinsic_mat(cv::Range(0,3),cv::Range(3,4));

    cv_rotation_mat       = cv_rotation_mat.t();
    cv_translate_vec   = -cv_rotation_mat * cv_translate_vec;

    //rotate the cloud
    Eigen::Matrix3d egmat_to_camera_rotation;
    cv::cv2eigen(cv_rotation_mat, egmat_to_camera_rotation);

    Eigen::Translation3d egmat_to_camera_translation(cv_translate_vec.at<double>(0,0),
                                            cv_translate_vec.at<double>(1,0),
                                            cv_translate_vec.at<double>(2,0));
    Eigen::Matrix4d egmat_to_camera_transformation = (egmat_to_camera_translation * egmat_to_camera_rotation).matrix();

    //filter
    std::vector<cv::Point3d> vec_3D_points;
    pcl::transformPointCloud (*inputPointCloud, *pcptr_rotated_point_cloud, egmat_to_camera_transformation);

    for (size_t i = 0; i < inputPointCloud->size(); ++i)
    {
        pcl::PointXYZRGB pt_3d_point = inputPointCloud->points[i]; // for filter
        pcl::PointXYZRGB pt_3d_point_transformed = pcptr_rotated_point_cloud->points[i]; // for rotation(depth color)
        if (pt_3d_point.x > -i_pointcloud_2d_projection_range_m_ && pt_3d_point.x < i_pointcloud_2d_projection_range_m_ && pt_3d_point.y > -i_pointcloud_2d_projection_range_m_ && pt_3d_point.y < i_pointcloud_2d_projection_range_m_ && pt_3d_point_transformed.z > 0)
        {
            vec_3D_points.emplace_back(cv::Point3d(pt_3d_point.x, pt_3d_point.y, pt_3d_point.z));
            double d_current_distance = pt_3d_point_transformed.z;

            d_max_range_projected_point_cloud_m_ = (d_current_distance > d_max_range_projected_point_cloud_m_)? d_current_distance: d_max_range_projected_point_cloud_m_;
            d_min_range_projected_point_cloud_m_ = (d_current_distance < d_min_range_projected_point_cloud_m_)? d_current_distance: d_min_range_projected_point_cloud_m_;
        }
    }

    // project 3d-points into image view
    if(vec_3D_points.size() > 0)
    {
        std::vector<cv::Point2d> vec_2d_projected_point;
        cv::projectPoints(vec_3D_points, cv_rotation_mat, cv_translate_vec, cv_camera_intrinsic_mat, cv_camera_distortion_mat, vec_2d_projected_point);

        vec_output_2d_projected_point = vec_2d_projected_point;
    }
}

void LandmarkMatchingPF::ParticlePointsTransformation(void)
{
    vec_particle_based_projected_points_building_.clear();
    vec_particle_based_projected_points_fence_.clear();
    vec_particle_based_projected_points_lane_marking_.clear();
    vec_particle_based_projected_points_trunk_.clear();
    vec_particle_based_projected_points_pole_.clear();
    vec_particle_based_projected_points_traffic_sign_.clear();
    vec_particle_based_projected_points_traffic_light_.clear();
    vec_particle_based_projected_points_tunnel_fan_.clear();
    vec_particle_based_projected_points_tunnel_light_.clear();
    vec_particle_based_projected_points_tunnel_hydrant_.clear();

    vec_ref_par_transform_.clear();

    Eigen::MatrixXd parPoint(3,1);
    Eigen::MatrixXd veh2parPoint(3,1);

    Eigen::Matrix3d veh2parRotation;

    Eigen::Matrix4d par2lidarTransformation;
    Eigen::Matrix4d ref2parTransformation;
    Eigen::Matrix4d egmat_cam_to_lidar_transform_mat;

    double d_reference_pose_x_m    = egmat_estimated_state_->coeff(0,0);
    double d_reference_pose_y_m    = egmat_estimated_state_->coeff(0,1);
    double d_reference_heading_rad = egmat_estimated_state_->coeff(0,2);

    std::vector<cv::Point2d> vec_projected_point_building;
    std::vector<cv::Point2d> vec_projected_point_fence;
    std::vector<cv::Point2d> vec_projected_point_lane_marking;
    std::vector<cv::Point2d> vec_projected_point_trunk;
    std::vector<cv::Point2d> vec_projected_point_pole;
    std::vector<cv::Point2d> vec_projected_point_traffic_sign;
    std::vector<cv::Point2d> vec_projected_point_traffic_light;
    std::vector<cv::Point2d> vec_projected_point_tunnel_fan;
    std::vector<cv::Point2d> vec_projected_point_tunnel_light;
    std::vector<cv::Point2d> vec_projected_point_tunnel_hydrant;
    
    for(int idx_particle = 0; idx_particle < param_i_num_particle; idx_particle++)
    {
        // 1. Get Ref <-> Particle Transformation    
        double d_particle_delta_east    = d_reference_pose_x_m - egmat_timestamped_particle_state_->coeff(idx_particle, 0);
        double d_particle_delta_north    = d_reference_pose_y_m - egmat_timestamped_particle_state_->coeff(idx_particle, 1);
        double d_particle_delta_heading = d_reference_heading_rad - egmat_timestamped_particle_state_->coeff(idx_particle, 2);

        ref2parTransformation << cos(d_particle_delta_heading), -sin(d_particle_delta_heading), 0., d_particle_delta_east,
                                 sin(d_particle_delta_heading),  cos(d_particle_delta_heading), 0., d_particle_delta_north,
                                 0.,                 0.,                1., 0.,
                                 0.,                 0.,                0., 1.; 

        // ref2parTransformation = egmat_lidar_vehicle_transform_ * ref2parTransformation;
        vec_ref_par_transform_.push_back(ref2parTransformation);

        PointCloud2DProjection(pcptr_building_sampled_point_cloud_,       ref2parTransformation, vec_projected_point_building);
        PointCloud2DProjection(pcptr_fence_sampled_point_cloud_,          ref2parTransformation, vec_projected_point_fence);
        PointCloud2DProjection(pcptr_lane_marking_sampled_point_cloud_,   ref2parTransformation, vec_projected_point_lane_marking);
        PointCloud2DProjection(pcptr_trunk_sampled_point_cloud_,          ref2parTransformation, vec_projected_point_trunk);
        PointCloud2DProjection(pcptr_pole_sampled_point_cloud_,           ref2parTransformation, vec_projected_point_pole);
        PointCloud2DProjection(pcptr_traffic_sign_sampled_point_cloud_,   ref2parTransformation, vec_projected_point_traffic_sign);
        PointCloud2DProjection(pcptr_traffic_light_sampled_point_cloud_,  ref2parTransformation, vec_projected_point_traffic_light);
        PointCloud2DProjection(pcptr_tunnel_fan_sampled_point_cloud_,     ref2parTransformation, vec_projected_point_tunnel_fan);
        PointCloud2DProjection(pcptr_tunnel_light_sampled_point_cloud_,   ref2parTransformation, vec_projected_point_tunnel_light);
        PointCloud2DProjection(pcptr_tunnel_hydrant_sampled_point_cloud_, ref2parTransformation, vec_projected_point_tunnel_hydrant);

        vec_particle_based_projected_points_building_.push_back(vec_projected_point_building);
        vec_particle_based_projected_points_fence_.push_back(vec_projected_point_fence);
        vec_particle_based_projected_points_lane_marking_.push_back(vec_projected_point_lane_marking);
        vec_particle_based_projected_points_trunk_.push_back(vec_projected_point_trunk);
        vec_particle_based_projected_points_pole_.push_back(vec_projected_point_pole);
        vec_particle_based_projected_points_traffic_sign_.push_back(vec_projected_point_traffic_sign);
        vec_particle_based_projected_points_traffic_light_.push_back(vec_projected_point_traffic_light);
        vec_particle_based_projected_points_tunnel_fan_.push_back(vec_projected_point_tunnel_fan);
        vec_particle_based_projected_points_tunnel_light_.push_back(vec_projected_point_tunnel_light);
        vec_particle_based_projected_points_tunnel_hydrant_.push_back(vec_projected_point_tunnel_hydrant);
    }

    //For reference points
    PointCloud2DProjection(pcptr_building_sampled_point_cloud_,       egmat_lidar_vehicle_transform_, vec_ref_pose_based_projected_point_building_);
    PointCloud2DProjection(pcptr_fence_sampled_point_cloud_,          egmat_lidar_vehicle_transform_, vec_ref_pose_based_projected_point_fence_);
    PointCloud2DProjection(pcptr_lane_marking_sampled_point_cloud_,   egmat_lidar_vehicle_transform_, vec_ref_pose_based_projected_point_lane_marking_);
    PointCloud2DProjection(pcptr_trunk_sampled_point_cloud_,          egmat_lidar_vehicle_transform_, vec_ref_pose_based_projected_point_trunk_);
    PointCloud2DProjection(pcptr_pole_sampled_point_cloud_,           egmat_lidar_vehicle_transform_, vec_ref_pose_based_projected_point_pole_);
    PointCloud2DProjection(pcptr_traffic_sign_sampled_point_cloud_,   egmat_lidar_vehicle_transform_, vec_ref_pose_based_projected_point_traffic_sign_);
    PointCloud2DProjection(pcptr_traffic_light_sampled_point_cloud_,  egmat_lidar_vehicle_transform_, vec_ref_pose_based_projected_point_traffic_light_);
    PointCloud2DProjection(pcptr_tunnel_fan_sampled_point_cloud_,     egmat_lidar_vehicle_transform_, vec_ref_pose_based_projected_point_tunnel_fan_);
    PointCloud2DProjection(pcptr_tunnel_light_sampled_point_cloud_,   egmat_lidar_vehicle_transform_, vec_ref_pose_based_projected_point_tunnel_light_);
    PointCloud2DProjection(pcptr_tunnel_hydrant_sampled_point_cloud_, egmat_lidar_vehicle_transform_, vec_ref_pose_based_projected_point_tunnel_hydrant_);
}

int LandmarkMatchingPF::CountNearbyIntersectionPoint(cv::Point2d cv_point, cv::Vec3b cvvec_ref_bgr, int d_bias)
{
    int d_bias_intersection_count = 0.;

    if(d_bias > 0)
    {
        for(int i_bias_y = -d_bias; i_bias_y <= d_bias; i_bias_y++)
        {
            cv::Vec3b cvvec_bgr = cvmat_segmented_image_.at<cv::Vec3b>(cv_point.y+i_bias_y, cv_point.x);

            if(cvvec_bgr == cvvec_ref_bgr)
            {
                d_bias_intersection_count ++;
            }

            cvvec_bgr = cvmat_segmented_image_.at<cv::Vec3b>(cv_point.y+i_bias_y, -cv_point.x);

            if(cvvec_bgr == cvvec_ref_bgr)
            {
                d_bias_intersection_count ++;
            }
        }

        for(int i_bias_x = -d_bias+1; i_bias_x < d_bias; i_bias_x++)
        {
            cv::Vec3b cvvec_bgr = cvmat_segmented_image_.at<cv::Vec3b>(cv_point.y, cv_point.x+i_bias_x);

            if(cvvec_bgr == cvvec_ref_bgr)
            {
                d_bias_intersection_count ++;
            }

            cvvec_bgr = cvmat_segmented_image_.at<cv::Vec3b>(-cv_point.y, cv_point.x+i_bias_x);

            if(cvvec_bgr == cvvec_ref_bgr)
            {
                d_bias_intersection_count ++;
            }
        }
    }

    return d_bias_intersection_count;
}

bool LandmarkMatchingPF::CheckNearbyIntersectionPointExist(cv::Point2d cv_point, cv::Vec3b cvvec_ref_bgr, int d_bias)
{
    if(d_bias > -1)
    {
        for(int i_bias_y = -d_bias; i_bias_y <= d_bias; i_bias_y++)
        {
            cv::Vec3b cvvec_bgr = cvmat_segmented_image_.at<cv::Vec3b>(cv_point.y+i_bias_y, cv_point.x);

            if(cvvec_bgr == cvvec_ref_bgr)
            {
                return true;
            }

            cvvec_bgr = cvmat_segmented_image_.at<cv::Vec3b>(cv_point.y+i_bias_y, -cv_point.x);

            if(cvvec_bgr == cvvec_ref_bgr)
            {
                return true;
            }
        }

        if(d_bias > 2)
        {
            for(int i_bias_x = -d_bias+1; i_bias_x < d_bias; i_bias_x++)
            {
                cv::Vec3b cvvec_bgr = cvmat_segmented_image_.at<cv::Vec3b>(cv_point.y, cv_point.x+i_bias_x);

                if(cvvec_bgr == cvvec_ref_bgr)
                {
                    return true;
                }

                cvvec_bgr = cvmat_segmented_image_.at<cv::Vec3b>(-cv_point.y, cv_point.x+i_bias_x);

                if(cvvec_bgr == cvvec_ref_bgr)
                {
                    return true;
                }
            }
        }
    }

    return false;
}

int LandmarkMatchingPF::CountPixels(const cv::Mat &image, cv::Vec3b cvvec_color) 
{
    cv::Mat binary_image;
    cv::inRange(image, cvvec_color, cvvec_color, binary_image);
        return cv::countNonZero(binary_image);
}

double LandmarkMatchingPF::ContourAreaCalc(std::vector<cv::Point> cv_point)
{
    double contourArea = cv::contourArea(cv_point);
    return contourArea;
}

double LandmarkMatchingPF::CheckPointsInsideContour(std::vector<cv::Point> cv_point, cv::Point2d cvp_check_point)
{
    double d_check_inside_contour = cv::pointPolygonTest(cv_point, cvp_check_point, false);
    return d_check_inside_contour;
}

double LandmarkMatchingPF::EigenMatrixSum(Eigen::MatrixXd* egmat_input_matrix)
{
    double d_sum = 0.;

    if(egmat_input_matrix->cols()>0 && egmat_input_matrix->rows()>0)
    {
        for(int i_col_idx = 0; i_col_idx < egmat_input_matrix->cols(); i_col_idx++)
        {
            for(int i_row_idx = 0; i_row_idx < egmat_input_matrix->rows(); i_row_idx++)
            {
                d_sum += egmat_input_matrix->coeff(i_row_idx, i_col_idx);
            }
        }

        return d_sum;
    }
    else
    {
        return 0.;
    }
    
}

// 4-3-2. VISUALIZATION FUNCTIONS
void LandmarkMatchingPF::ReferenceImagePointViewer(std::vector<cv::Point2d> vec_input_points)
{
    cv::Mat cvmat_image_origin = cvptr_input_image_->image;
    cv::Mat cvmat_image_projection = cvmat_image_origin.clone();

    int i_image_rows = cvmat_image_origin.rows;
    int i_image_cols = cvmat_image_origin.cols;
    
    for (size_t i = 0; i < vec_input_points.size(); ++i)
    {
        cv::Point2d cvp_point_2d = vec_input_points[i];
        cv::rectangle(cvmat_image_projection,cv::Point(cvp_point_2d.x ,cvp_point_2d.y ),
                                        cv::Point(cvp_point_2d.x ,cvp_point_2d.y ),
                                        CV_RGB(255,0,0),
                                        3);
    }
    cv::imshow("Ref Whole Point Cloud Image", cvmat_image_projection);
    cv::waitKey(1);
}

void LandmarkMatchingPF::ReferenceSegmentedImagePointAndCheckInsideViewer()
{
    vec_inside_segmented_points_.clear();
    vec_near_segmented_points_.clear();
    vec_outside_segmented_points_.clear();

    cv::Mat cvmat_image_origin = cvmat_segmented_image_;

    int i_image_rows = cvmat_image_origin.rows;
    int i_image_cols = cvmat_image_origin.cols;

    double d_likelihood_building = SegmentedImageSPCMatchingVisualization(vec_ref_pose_based_projected_point_building_, cvvec_building_semantic_rgb_, 2);
    double d_likelihood_lane_marking = SegmentedImageSPCMatchingVisualization(vec_ref_pose_based_projected_point_lane_marking_, cvvec_lane_semantic_rgb_, 16);
    double d_likelihood_fence = SegmentedImageSPCMatchingVisualization(vec_ref_pose_based_projected_point_fence_, cvvec_fence_semantic_rgb_, 2);

    std::string str_building = "Builing Score : " + std::to_string(d_likelihood_building);
    std::string str_lane_marking = "Lane Marking Score : " + std::to_string(d_likelihood_lane_marking);
    std::string str_fence = "Fence Score : " + std::to_string(d_likelihood_fence);

    cv::putText(cvmat_image_origin, str_building, cv::Point(40, cvmat_image_origin.rows-30), cv::FONT_HERSHEY_DUPLEX, 1.0, CV_RGB(255,128,255), 2);
    cv::putText(cvmat_image_origin, str_lane_marking, cv::Point(40, cvmat_image_origin.rows-60), cv::FONT_HERSHEY_DUPLEX, 1.0, CV_RGB(128,128,255), 2);
    cv::putText(cvmat_image_origin, str_fence, cv::Point(40, cvmat_image_origin.rows-90), cv::FONT_HERSHEY_DUPLEX, 1.0, CV_RGB(255,128,128), 2);

    for (int i = 0; i < vec_outside_segmented_points_.size(); i++)
    {
        cv::Point2d cvp_point_2d = vec_outside_segmented_points_[i];
        cv::rectangle(cvmat_image_origin,cv::Point(cvp_point_2d.x ,cvp_point_2d.y ),
                                        cv::Point(cvp_point_2d.x ,cvp_point_2d.y ),
                                        CV_RGB(255,255,255),
                                        3);
    }

    for (int i = 0; i < vec_inside_segmented_points_.size(); i++)
    {
        cv::Point2d cvp_point_2d = vec_inside_segmented_points_[i];
        cv::rectangle(cvmat_image_origin,cv::Point(cvp_point_2d.x-16 ,cvp_point_2d.y-16 ),
                                        cv::Point(cvp_point_2d.x+16 ,cvp_point_2d.y+16 ),
                                        CV_RGB(255,255,0),
                                        2.0);
    }

    for(int i = 0; i < vec_near_segmented_points_.size(); i++)
    {
        cv::Point2d cvp_point_2d = vec_near_segmented_points_[i];

        cv::rectangle(cvmat_image_origin,cv::Point(cvp_point_2d.x-8 ,cvp_point_2d.y-8 ),
                                        cv::Point(cvp_point_2d.x+8 ,cvp_point_2d.y+8 ),
                                        CV_RGB(128,255,255),
                                        1.5);
    }

    cv::imshow("Ref Segmented Point Cloud Image", cvmat_image_origin);
    cv::waitKey(1);
}

void LandmarkMatchingPF::ReferenceImagePointAndCheckInsideViewer()
{
    vec_inside_contour_points_.clear();
    vec_outside_contour_points_.clear();

    double d_likelihood_pole          = ImageSPCMatchingVisualization(vec_image_contour_point_pole_, vec_ref_pose_based_projected_point_pole_);
    double d_likelihood_traffic_sign   = ImageSPCMatchingVisualization(vec_image_contour_point_traffic_sign_, vec_ref_pose_based_projected_point_traffic_sign_);
    double d_likelihood_traffic_light  = ImageSPCMatchingVisualization(vec_image_contour_point_traffic_light_, vec_ref_pose_based_projected_point_traffic_light_);
    double d_likelihood_tunnel_fan     = ImageSPCMatchingVisualization(vec_image_contour_point_tunnel_fan_, vec_ref_pose_based_projected_point_tunnel_fan_);
    double dLikelihoodTunnelLight   = ImageSPCMatchingVisualization(vec_image_contour_point_tunnel_light_, vec_ref_pose_based_projected_point_tunnel_light_);
    double d_likelihood_tunnel_hydrant = ImageSPCMatchingVisualization(vec_image_contour_point_tunnel_hydrant, vec_ref_pose_based_projected_point_tunnel_hydrant_);

    cv::Mat cvmat_image_origin  = cvptr_input_image_->image;
    cv::Mat cvmat_image_projection = cvmat_image_origin.clone();

    int i_image_rows = cvmat_image_origin.rows;
    int i_image_cols = cvmat_image_origin.cols;
    
    for (size_t i = 0; i < vec_inside_contour_points_.size(); ++i)
    {
        cv::Point2d cvp_point_2d = vec_inside_contour_points_[i];
            
        cv::rectangle(cvmat_image_projection,cv::Point(cvp_point_2d.x, cvp_point_2d.y),
                                    cv::Point(cvp_point_2d.x, cvp_point_2d.y),
                                    CV_RGB(255,0,0),
                                    15);
    }

    for (size_t i = 0; i < vec_outside_contour_points_.size(); ++i)
    {
        cv::Point2d cvp_point_2d = vec_outside_contour_points_[i];
            
        cv::rectangle(cvmat_image_projection,cv::Point(cvp_point_2d.x, cvp_point_2d.y),
                                    cv::Point(cvp_point_2d.x, cvp_point_2d.y),
                                    CV_RGB(0,0,255),
                                    3);
    }

    std::string str_pole          = "Pole Score : "           + std::to_string(d_likelihood_pole);
    std::string str_traffic_sign   = "Traffic Sign Score : "   + std::to_string(d_likelihood_traffic_sign);
    std::string str_traffic_light  = "Traffic light Score : "  + std::to_string(d_likelihood_traffic_light);
    std::string str_tunnel_fan     = "Tunnel Fan Score : "     + std::to_string(d_likelihood_tunnel_fan);
    std::string str_tunnel_light   = "Tunnel Light Score : "   + std::to_string(dLikelihoodTunnelLight);
    std::string str_tunnel_hydrant = "Tunnel Hydrant Score : " + std::to_string(d_likelihood_tunnel_hydrant);

    cv::putText(cvmat_image_projection, str_pole,          cv::Point(40, cvmat_image_projection.rows-10),  cv::FONT_HERSHEY_DUPLEX, 1.0, CV_RGB(0,255,0),   2);
    cv::putText(cvmat_image_projection, str_traffic_sign,   cv::Point(40, cvmat_image_projection.rows-38),  cv::FONT_HERSHEY_DUPLEX, 1.0, CV_RGB(255,255,0), 2);
    cv::putText(cvmat_image_projection, str_traffic_light,  cv::Point(40, cvmat_image_projection.rows-63),  cv::FONT_HERSHEY_DUPLEX, 1.0, CV_RGB(0,255,255), 2);
    cv::putText(cvmat_image_projection, str_tunnel_fan,     cv::Point(40, cvmat_image_projection.rows-88),  cv::FONT_HERSHEY_DUPLEX, 1.0, CV_RGB(255,0,0),   2);
    cv::putText(cvmat_image_projection, str_tunnel_light,   cv::Point(40, cvmat_image_projection.rows-113), cv::FONT_HERSHEY_DUPLEX, 1.0, CV_RGB(0,0,255),   2);
    cv::putText(cvmat_image_projection, str_tunnel_hydrant, cv::Point(40, cvmat_image_projection.rows-138), cv::FONT_HERSHEY_DUPLEX, 1.0, CV_RGB(255,0,255), 2);

    if(vec_image_contour_point_pole_.size() != 0) //str_pole
        cv::drawContours(cvmat_image_projection, vec_image_contour_point_pole_, -1, CV_RGB(0,255,0),   2, 8);
    if(vec_image_contour_point_traffic_sign_.size() != 0) // traffic sign
        cv::drawContours(cvmat_image_projection, vec_image_contour_point_traffic_sign_, -1, CV_RGB(255,255,0), 2, 8);
    if(vec_image_contour_point_traffic_light_.size() != 0) // traffic light
        cv::drawContours(cvmat_image_projection, vec_image_contour_point_traffic_light_, -1, CV_RGB(0,255,255), 2, 8);
    if(vec_image_contour_point_tunnel_fan_.size() != 0) // tunnel fan
        cv::drawContours(cvmat_image_projection, vec_image_contour_point_tunnel_fan_, -1, CV_RGB(255,0,0),   2, 8);
    if(vec_image_contour_point_tunnel_light_.size() != 0) // tunnel light
        cv::drawContours(cvmat_image_projection, vec_image_contour_point_tunnel_light_, -1, CV_RGB(0,0,255),   2, 8);
    if(vec_image_contour_point_tunnel_hydrant.size() != 0) // tunnel hydrant
        cv::drawContours(cvmat_image_projection, vec_image_contour_point_tunnel_hydrant, -1, CV_RGB(255,0,255), 2, 8);

    cv::imshow("Ref Image", cvmat_image_projection);
    cv::waitKey(1);
}

void LandmarkMatchingPF::ParticleImagePointViewer(std::vector<cv::Point2d> vec_input_points, int i_particle_num)
{
    if(i_particle_num > param_i_num_particle || i_particle_num < 0)
    {
        ROS_WARN_STREAM("INVALID PARTICLE NUMBER ! COULD NOT VISUALIZE !");
        return;
    }

    vec_inside_contour_points_.clear();
    vec_outside_contour_points_.clear();

    double d_likelihood_pole          = ImageSPCMatchingVisualization(vec_image_contour_point_pole_, vec_particle_based_projected_points_pole_[i_particle_num]);
    double d_likelihood_traffic_sign   = ImageSPCMatchingVisualization(vec_image_contour_point_traffic_sign_, vec_particle_based_projected_points_traffic_sign_[i_particle_num]);
    double d_likelihood_traffic_light  = ImageSPCMatchingVisualization(vec_image_contour_point_traffic_light_, vec_particle_based_projected_points_traffic_light_[i_particle_num]);
    double d_likelihood_tunnel_fan     = ImageSPCMatchingVisualization(vec_image_contour_point_tunnel_fan_, vec_particle_based_projected_points_tunnel_fan_[i_particle_num]);
    double dLikelihoodTunnelLight   = ImageSPCMatchingVisualization(vec_image_contour_point_tunnel_light_, vec_particle_based_projected_points_tunnel_light_[i_particle_num]);
    double d_likelihood_tunnel_hydrant = ImageSPCMatchingVisualization(vec_image_contour_point_tunnel_hydrant, vec_particle_based_projected_points_tunnel_hydrant_[i_particle_num]);

    cv::Mat cvmat_image_origin  = cvptr_input_image_->image;
    cv::Mat cvmat_image_projection = cvmat_image_origin.clone();

    int i_image_rows = cvmat_image_origin.rows;
    int i_image_cols = cvmat_image_origin.cols;
    
    for (size_t i = 0; i < vec_inside_contour_points_.size(); ++i)
    {
        cv::Point2d cvp_point_2d = vec_inside_contour_points_[i];
            
        cv::rectangle(cvmat_image_projection,cv::Point(cvp_point_2d.x, cvp_point_2d.y),
                                    cv::Point(cvp_point_2d.x, cvp_point_2d.y),
                                    CV_RGB(255,0,0),
                                    15);
    }

    for (size_t i = 0; i < vec_outside_contour_points_.size(); ++i)
    {
        cv::Point2d cvp_point_2d = vec_outside_contour_points_[i];
            
        cv::rectangle(cvmat_image_projection,cv::Point(cvp_point_2d.x, cvp_point_2d.y),
                                    cv::Point(cvp_point_2d.x, cvp_point_2d.y),
                                    CV_RGB(0,0,255),
                                    3);
    }

    std::string str_pole          = "Pole Score : "           + std::to_string(d_likelihood_pole);
    std::string str_traffic_sign   = "Traffic Sign Score : "   + std::to_string(d_likelihood_traffic_sign);
    std::string str_traffic_light  = "Traffic light Score : "  + std::to_string(d_likelihood_traffic_light);
    std::string str_tunnel_fan     = "Tunnel Fan Score : "     + std::to_string(d_likelihood_tunnel_fan);
    std::string str_tunnel_light   = "Tunnel Light Score : "   + std::to_string(dLikelihoodTunnelLight);
    std::string str_tunnel_hydrant = "Tunnel Hydrant Score : " + std::to_string(d_likelihood_tunnel_hydrant);

    std::string particleNum   = "Particle Num : "+std::to_string(i_particle_num);

    cv::putText(cvmat_image_projection, str_pole,          cv::Point(40, cvmat_image_projection.rows-10),  cv::FONT_HERSHEY_DUPLEX, 1.0, CV_RGB(0,255,0),   2);
    cv::putText(cvmat_image_projection, str_traffic_sign,   cv::Point(40, cvmat_image_projection.rows-38),  cv::FONT_HERSHEY_DUPLEX, 1.0, CV_RGB(255,255,0), 2);
    cv::putText(cvmat_image_projection, str_traffic_light,  cv::Point(40, cvmat_image_projection.rows-63),  cv::FONT_HERSHEY_DUPLEX, 1.0, CV_RGB(0,255,255), 2);
    cv::putText(cvmat_image_projection, str_tunnel_fan,     cv::Point(40, cvmat_image_projection.rows-88),  cv::FONT_HERSHEY_DUPLEX, 1.0, CV_RGB(255,0,0),   2);
    cv::putText(cvmat_image_projection, str_tunnel_light,   cv::Point(40, cvmat_image_projection.rows-113), cv::FONT_HERSHEY_DUPLEX, 1.0, CV_RGB(0,0,255),   2);
    cv::putText(cvmat_image_projection, str_tunnel_hydrant, cv::Point(40, cvmat_image_projection.rows-138), cv::FONT_HERSHEY_DUPLEX, 1.0, CV_RGB(255,0,255), 2);

    cv::putText(cvmat_image_projection, particleNum,   cv::Point(50, 30),                     cv::FONT_HERSHEY_DUPLEX, 1.0, CV_RGB(0,0,0),     2);

    if(vec_image_contour_point_pole_.size() != 0) //str_pole
        cv::drawContours(cvmat_image_projection, vec_image_contour_point_pole_, -1, CV_RGB(0,255,0), 2, 8);
    if(vec_image_contour_point_traffic_sign_.size() != 0) // traffic sign
        cv::drawContours(cvmat_image_projection, vec_image_contour_point_traffic_sign_, -1, CV_RGB(255,255,0), 2, 8);
    if(vec_image_contour_point_traffic_light_.size() != 0) // traffic light
        cv::drawContours(cvmat_image_projection, vec_image_contour_point_traffic_light_, -1, CV_RGB(0,255,255), 2, 8);
    if(vec_image_contour_point_tunnel_fan_.size() != 0) // tunnel fan
        cv::drawContours(cvmat_image_projection, vec_image_contour_point_tunnel_fan_, -1, CV_RGB(255,0,0), 2, 8);
    if(vec_image_contour_point_tunnel_light_.size() != 0) // tunnel light
        cv::drawContours(cvmat_image_projection, vec_image_contour_point_tunnel_light_, -1, CV_RGB(0,0,255), 2, 8);
    if(vec_image_contour_point_tunnel_hydrant.size() != 0) // tunnel hydrant
        cv::drawContours(cvmat_image_projection, vec_image_contour_point_tunnel_hydrant, -1, CV_RGB(255,0,255), 2, 8);

    std::string viewer = "Particle " + std::to_string(i_particle_num);
    cv::imshow(viewer, cvmat_image_projection);
    cv::waitKey(1);
}

double LandmarkMatchingPF::ImageSPCMatchingVisualization(std::vector<std::vector<cv::Point>> vec_input_contours, std::vector<cv::Point2d> vec_input_points)
{
    // Intersection = # of intersection points * (image area / num of all points)
    if(vec_input_contours.size()==0 || vec_input_points.size() == 0)
    {
        return 0.;
    }

    int i_intersection_points_count = 0;

    for(auto& contour : vec_input_contours)
    {
        for(auto& point : vec_input_points)
        {
            if(CheckPointsInsideContour(contour, point)>=0.)
            {
                i_intersection_points_count++;
                vec_inside_contour_points_.push_back(point);
            }
            else
            {
                vec_outside_contour_points_.push_back(point);
            }
        }
    }

    double d_intersection_score = (double)i_intersection_points_count;

    return d_intersection_score;
}

double LandmarkMatchingPF::SegmentedImageSPCMatchingVisualization(std::vector<cv::Point2d> vec_input_points, cv::Vec3b cvvec_ref_bgr, int i_pixel_bias)
{
    if(vec_input_points.size() == 0 || cvmat_segmented_image_.empty())
    {
        return 0.;
    }

    double i_intersection_points_count = 0.;
    
    for(auto& point : vec_input_points)
    {
        if((int)point.x < cvmat_segmented_image_.cols && (int)point.x > 0.
            && (int)point.y < cvmat_segmented_image_.rows && (int)point.y > 0.)
        {
            for(int pixel_idx = 0; pixel_idx < i_pixel_bias; pixel_idx++)
            {
                if(CheckNearbyIntersectionPointExist(point, cvvec_ref_bgr, pixel_idx))
                {
                    if(pixel_idx < 1)
                    {
                        vec_inside_segmented_points_.push_back(point);
                    }
                    else
                    {
                        vec_near_segmented_points_.push_back(point);
                    }
                    
                    i_intersection_points_count =  i_intersection_points_count + (1.0 - pixel_idx*0.05);
                    break;
                }
                else
                {
                    vec_outside_segmented_points_.push_back(point);
                }
            }
        }
    }
    
    double d_intersection_score = (double)i_intersection_points_count;
    return d_intersection_score;
}

// ETC
geometry_msgs::PoseStamped LandmarkMatchingPF::ConvertToMapFrame(float f_lat, float f_lon, float f_hgt)
{
    double d_kappa_lat = 0.;
    double d_kappa_lon = 0.;  

    double d_ref_latitude_deg = 37.3962732790;
    double d_ref_longitude_deg = 127.1066872418;

    f_hgt = d_map_height_;

    double d_denominator = sqrt(1. - D_GEOD_E2 * pow(sin(d_ref_latitude_deg * D_DEG_2_RAD), 2));
	double dM = D_GEOD_A * (1. - D_GEOD_E2) / pow(d_denominator, 3);

	// Curvature for the meridian
	d_kappa_lat = 1. / (dM + f_hgt) * D_RAD_2_DEG;

	// estimate the normal radius
	d_denominator = sqrt(1. - D_GEOD_E2 * pow(sin(d_ref_latitude_deg * D_DEG_2_RAD), 2));
	double d_dn = D_GEOD_A / d_denominator;

	// Curvature for the meridian
	d_kappa_lon = 1. / ((d_dn + f_hgt) * cos(d_ref_latitude_deg * D_DEG_2_RAD)) * D_RAD_2_DEG;

    geometry_msgs::PoseStamped psstp_pose;
    geometry_msgs::Quaternion gquat_quaternion;

    psstp_pose.header.stamp = ros::Time::now();
    psstp_pose.header.frame_id = "map";

    psstp_pose.pose.position.x = (f_lon-d_ref_longitude_deg)/d_kappa_lon;
    psstp_pose.pose.position.y = (f_lat-d_ref_latitude_deg)/d_kappa_lat;
    psstp_pose.pose.position.z = f_hgt;

    return(psstp_pose);
}

double LandmarkMatchingPF::GaussianRandomGenerator(double d_mean, double d_sigma)
{
	if (d_sigma > FLT_MIN)
	{
		std::normal_distribution<double> nordist_normal_distribution(d_mean, d_sigma);
		return nordist_normal_distribution(random_generator_);
	}
	else
	{
		return d_mean;
	}
}

void LandmarkMatchingPF::SaveResultText(Eigen::MatrixXd* input, std::string mathcingType)
{
    std::ofstream resultText(strmsg_result_path_, std::ofstream::out | std::ofstream::app);

    auto timeNow = std::chrono::high_resolution_clock::now();
    auto duration_prediction = std::chrono::duration_cast<std::chrono::milliseconds>( timeNow- m_init_time ).count();

    double time = (double)duration_prediction;

    for(int idxPar = 0; idxPar < param_i_num_particle; idxPar++)
    {
        resultText  <<std::to_string(time)<< ", "<<mathcingType<<", "<<std::to_string(idxPar)<<", "<<std::to_string(egmat_timestamped_particle_state_->coeff(idxPar, 0))<<", "<<std::to_string(egmat_timestamped_particle_state_->coeff(idxPar, 1))
                    << ", "<<std::to_string(egmat_timestamped_particle_state_->coeff(idxPar, 2))<<", "<<std::to_string(psstp_gnss_enu_pose_.pose.position.x)<<", "<<std::to_string(psstp_gnss_enu_pose_.pose.position.y)
                    << ", "<<std::to_string(gps_gps_data_.yaw) << ", "<< std::to_string(input->coeff(idxPar, 0))<<"\n";
    }
        
    i_frame_count++;
}

void LandmarkMatchingPF::PublishParticlePoseArray(void)
{
    // Publish PoseArray
    geometry_msgs::PoseArray psarray_particle_pose_array;
    psarray_particle_pose_array.header.frame_id = "map";
    psarray_particle_pose_array.header.stamp = ros::Time::now();

    for(int idx_particle = 0; idx_particle < param_i_num_particle; idx_particle++)
    {
        geometry_msgs::Pose pose_particle_pose;

        pose_particle_pose.position.x = egmat_timestamped_particle_state_->coeff(idx_particle, 0);
        pose_particle_pose.position.y = egmat_timestamped_particle_state_->coeff(idx_particle, 1);
        // sin,cos vs d_reference_heading_rad
        pose_particle_pose.position.z = d_map_height_;

        tf::Quaternion tfquat_quaternion;
        tfquat_quaternion.setRPY(0, 0, egmat_timestamped_particle_state_->coeff(idx_particle, 2));

        geometry_msgs::Quaternion gquat_quaternion;
        tf::quaternionTFToMsg(tfquat_quaternion, gquat_quaternion);

        pose_particle_pose.orientation = gquat_quaternion;

        psarray_particle_pose_array.poses.push_back(pose_particle_pose);
    }
    rospub_particle_pose_array_.publish(psarray_particle_pose_array);

    //Publish Marker
    visualization_msgs::MarkerArray vmkarray_particle_marker_array;

    for(int idx_particle = 0; idx_particle < param_i_num_particle; idx_particle++)
    {
        visualization_msgs::Marker vmk_marker;
        vmk_marker.header.frame_id = "/map";
        vmk_marker.header.stamp = ros::Time::now();

        vmk_marker.ns = "particle";
        vmk_marker.id = idx_particle;

        vmk_marker.type = visualization_msgs::Marker::ARROW;
        vmk_marker.action = visualization_msgs::Marker::ADD;

        vmk_marker.pose.position.x = egmat_timestamped_particle_state_->coeff(idx_particle, 0);
        vmk_marker.pose.position.y = egmat_timestamped_particle_state_->coeff(idx_particle, 1);
        vmk_marker.pose.position.z = d_map_height_;

        tf::Quaternion tfquat_quaternion;
        tfquat_quaternion.setRPY(0, 0, egmat_timestamped_particle_state_->coeff(idx_particle, 2));

        geometry_msgs::Quaternion gquat_quaternion;
        tf::quaternionTFToMsg(tfquat_quaternion, gquat_quaternion);

        vmk_marker.pose.orientation = gquat_quaternion;

        vmk_marker.scale.x = (egmat_particle_weight_->coeff(idx_particle,0) * param_i_num_particle)*0.5;
        vmk_marker.scale.y = (egmat_particle_weight_->coeff(idx_particle,0) * param_i_num_particle)*0.5;
        vmk_marker.scale.z = (egmat_particle_weight_->coeff(idx_particle,0) * param_i_num_particle)*0.5;

        vmk_marker.color.r = 1.0f;
        vmk_marker.color.g = 1.0f;
        vmk_marker.color.b = 0.0f;
        vmk_marker.color.a = 1.0f;

        vmk_marker.lifetime = ros::Duration();

        vmkarray_particle_marker_array.markers.push_back(vmk_marker);
    }
    rospub_particle_marker_array_.publish(vmkarray_particle_marker_array);
}

int main(int argc, char **argv)
{
    std::string str_nodename = "landmark_matching_pf";
	ros::init(argc, argv, str_nodename);
	ros::NodeHandle nh;

    LandmarkMatchingPF LandmarkMatchingPF;
    LandmarkMatchingPF.Initialization();

    int i_init_loop_frequency = 10;

    ros::Rate rosrate_loop_rate(i_init_loop_frequency);

    while(ros::ok())
    {
            ros::spinOnce();
            LandmarkMatchingPF.Run();
            rosrate_loop_rate.sleep();
    }

    return 0;
}