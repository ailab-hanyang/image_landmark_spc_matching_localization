/*
 * @copyright Automotive Intelligence Lab, Konkuk University
 * @author pauljiwon96@gmail.com, kimchuorok@gmail.com, yondoo20@gmail.com 
 * @file spc_map_processing.cpp
 * @brief Image Landmark and Semantic Point Cloud Map Matching Localization
 * @version 1.0
 * @date 2021-03-25
 */

#include "spc_map_processing.hpp"

SPCMapProcessing::SPCMapProcessing()
:pcptr_point_cloud_intensity_(new pcl::PointCloud<pcl::PointXYZI>), pcptr_point_cloud_rgb_(new pcl::PointCloud<pcl::PointXYZRGB>), d_height_m_(0.)
{
    if(param_DEBUG_MODE_)
    {
        std::cout<<"[MAP PUBLISHER] Initialization start"<<std::endl;
    }
    if(m_rosNodeHandler.getParam("/spc_map_processing/param_str_pcd_file_path_", param_str_pcd_file_path_)){
        ROS_INFO("GET param_str_pcd_file_path_");
    }
    m_rosNodeHandler.getParam("/spc_map_processing/param_d_roi_x_boundary_length_m_", param_d_roi_x_boundary_length_m_);
    m_rosNodeHandler.getParam("/spc_map_processing/param_d_roi_y_boundary_length_m_", param_d_roi_y_boundary_length_m_);
    m_rosNodeHandler.getParam("/spc_map_processing/param_DEBUG_MODE_", param_DEBUG_MODE_);
    m_rosNodeHandler.getParam("/spc_map_processing/param_height_detection_type_minimum_", param_height_detection_type_minimum_);

    int i_buffer_size = 10;

    // Publisher Init
    rospub_map_rgb_ =
            m_rosNodeHandler.advertise<sensor_msgs::PointCloud2>("/global_map_RGB", i_buffer_size);
    rospub_map_pose_ =
            m_rosNodeHandler.advertise<geometry_msgs::Pose>("/map_height", 1);
    rospub_local_map_ = 
            m_rosNodeHandler.advertise<sensor_msgs::PointCloud2>("/local_map", 1);
    rospub_roi_map_ =
            m_rosNodeHandler.advertise<sensor_msgs::PointCloud2>("/roi_map", 1);
    rospub_roi_height_map_ = 
            m_rosNodeHandler.advertise<sensor_msgs::PointCloud2>("/roi_height_map", 1);
    rospub_roi_transformed_map_ =
            m_rosNodeHandler.advertise<sensor_msgs::PointCloud2>("/roi_transformedmap", 1);            
    // Subscirber Init
    rossub_ego_pose_ =
            m_rosNodeHandler.subscribe("/particleEstimatedPose", i_buffer_size, &SPCMapProcessing::PoseCallback, this);


    MapReaderFromPCDToPCLXYZ(*pcptr_point_cloud_rgb_);
    // MapReaderFromPCDToPCLXYZ(*pcptr_point_cloud_intensity_);
    PCL2pointcloud2(*pcptr_point_cloud_rgb_,pc2_point_cloud_rgb_);
    // PCL2pointcloud2(*pcptr_point_cloud_intensity_,pc2_point_cloud_intensity_);

    BuildKdtree();

    if(param_DEBUG_MODE_)
    {
        std::cout<<"[MAP PUBLISHER] Initialization finish"<<std::endl;
    }

}

SPCMapProcessing::~SPCMapProcessing()
{

}

void SPCMapProcessing::PoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    psstp_curr_enu_pose_ = *msg;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcptr_point_cloud_trimmed(new pcl::PointCloud<pcl::PointXYZRGB>);
    GetPointCloudROI(param_d_roi_x_boundary_length_m_, param_d_roi_y_boundary_length_m_, pcptr_point_cloud_trimmed);

    if(pcptr_point_cloud_trimmed->points.size()>6)
    {
        if(checker_b_is_init_height_ == false)
        {
            double d_min_height = GetLowestHeight(pcptr_point_cloud_trimmed);
            if(d_min_height != FLT_MAX)
            {
                d_height_m_ = d_min_height;
            }
            checker_b_is_init_height_ = true;
        }
        
        if(param_height_detection_type_minimum_)
        {
            double d_min_height = GetLowestHeight(pcptr_point_cloud_trimmed);
            if(d_height_m_ - d_min_height < 0.5 && d_min_height != FLT_MAX)
            {
                d_height_m_ = d_min_height;
            }
        }
        else
        {
            double d_ave_height = GetAverageHeight(pcptr_point_cloud_trimmed);
            if(d_height_m_ - d_ave_height < 0.5)
                d_height_m_ = d_ave_height;
        }
        geometry_msgs::Pose pose_map_height_rp = GetMapBasedRollPitch(pcptr_point_cloud_trimmed);
        pose_map_height_rp.position.z = d_height_m_;
        psstp_curr_enu_pose_.pose.orientation = pose_map_height_rp.orientation;
        
        rospub_map_pose_.publish(pose_map_height_rp);
    }
    
    ROIMapPublisher();
}

void SPCMapProcessing::ROIMapPublisher(void)
{
    if(kdtree_map_ == NULL)
    {
        ROS_WARN_STREAM("KD TREE MAP IS EMPTY!");
        return;
    }
    if(param_DEBUG_MODE_)
    {
        std::cout<<"[MAP PUBLISHER] ROIMap pub start"<<std::endl;
    }
    pcl::PointXYZRGB pt_search_point;
    pt_search_point.x = psstp_curr_enu_pose_.pose.position.x;
    pt_search_point.y = psstp_curr_enu_pose_.pose.position.y;
    pt_search_point.z = psstp_curr_enu_pose_.pose.position.z;

    float f_radius_m = 100.0;
    std::vector<int> vec_point_idx_radius_search;
    std::vector<float> vec_point_radius_squared_distance;

    kdtree_map_->radiusSearch(pt_search_point, f_radius_m, vec_point_idx_radius_search, vec_point_radius_squared_distance);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcptr_tmp_point_cloud_roi(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcptr_tmp_point_cloud_roi->header = pcptr_point_cloud_rgb_->header;

    for(int i_search_idx = 0; i_search_idx < vec_point_idx_radius_search.size(); i_search_idx++)
    {
        pcl::PointXYZRGB pt_tmp_roi_point_cloud;
        int i_point_idx = vec_point_idx_radius_search[i_search_idx];

        pt_tmp_roi_point_cloud = pcptr_point_cloud_rgb_->points[i_point_idx];

        pcptr_tmp_point_cloud_roi->points.push_back(pt_tmp_roi_point_cloud);
    }

    sensor_msgs::PointCloud2 pc2_roi_point_cloud;
    PCL2pointcloud2(*pcptr_tmp_point_cloud_roi,pc2_roi_point_cloud);

    //============================================================ ROI_PointCloud map generatot 

    pc2_roi_point_cloud.header.frame_id = "map";

    rospub_roi_map_.publish(pc2_roi_point_cloud);

    //============================================================ ROI_transformed map generator

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcptr_ego_vehicle_around_point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    sensor_msgs::PointCloud2 pc2_transformed_point_cloud;

    static tf::TransformListener tflisten_ego_velodyne;
    tf::StampedTransform tfstp_transform;
    try{
        tflisten_ego_velodyne.lookupTransform("estimated_pose", pc2_roi_point_cloud.header.frame_id, ros::Time(0), tfstp_transform);
    }
    catch(tf::TransformException tfexp_ex){
        ROS_ERROR("%s",tfexp_ex.what());
    }
    
    pcl_ros::transformPointCloud(*pcptr_tmp_point_cloud_roi,*pcptr_ego_vehicle_around_point_cloud,tfstp_transform);
    pcl::toROSMsg(*pcptr_ego_vehicle_around_point_cloud, pc2_transformed_point_cloud);
    pc2_transformed_point_cloud.header.frame_id = "estimated_pose";

    rospub_roi_transformed_map_.publish(pc2_transformed_point_cloud);
}
template<typename T>
bool SPCMapProcessing::MapReaderFromPCDToPCLXYZ(pcl::PointCloud<T> &input)
{
    if(param_DEBUG_MODE_)
    {
        std::cout<<"[MAP PUBLISHER] MapReader start"<<std::endl;
    }
    if(pcl::io::loadPCDFile<T> (param_str_pcd_file_path_.c_str(), input)==-1)
    {
        ROS_ERROR("PCD File read Error");
        return(-1);
    }
    if(param_DEBUG_MODE_)
    {
        std::cout<<"[MAP PUBLISHER] MapReader finish"<<std::endl;
    }
}
bool SPCMapProcessing::MapReaderFromPCDToPointcloud2(pcl::PCLPointCloud2 &input)
{
    if(param_DEBUG_MODE_)
    {
        std::cout<<"[MAP PUBLISHER] MapReader start"<<std::endl;
    }
    if(pcl::io::loadPCDFile (param_str_pcd_file_path_.c_str(), input)==-1)
    {
        ROS_ERROR("PCD File read Error");
        return(-1);
    }
    if(param_DEBUG_MODE_)
    {
        std::cout<<"[MAP PUBLISHER] MapReader finish"<<std::endl;
    }
}

template<typename T>
bool SPCMapProcessing::PCL2pointcloud2(pcl::PointCloud<T> &input, sensor_msgs::PointCloud2 &pc2_point_cloud_rgb)
{   
    if(param_DEBUG_MODE_)
    {
        std::cout<<"[MAP PUBLISHER] Convert PCL2pointcloud2 start"<<std::endl;
    }
    pcl::toROSMsg(input, pc2_point_cloud_rgb);
    pc2_point_cloud_rgb.header.frame_id = FRAMEID;
    if(param_DEBUG_MODE_)
    {
        std::cout<<"[MAP PUBLISHER] Convert PCL2pointcloud2 finish"<<std::endl;
    }
}

bool SPCMapProcessing::BuildKdtree(void)
{
    // Error checking
    if (!pcptr_point_cloud_rgb_) {
        return false;
    }
    
    // Kdtree generation
    pcl::KdTreeFLANN<pcl::PointXYZRGB>::Ptr kdtree_3d_map(new pcl::KdTreeFLANN<pcl::PointXYZRGB>);
    kdtree_3d_map->setInputCloud(pcptr_point_cloud_rgb_);
    kdtree_map_ = kdtree_3d_map;

    // 2D Kdtree generation
    pcl::PointCloud<pcl::PointXY>::Ptr pcptr_2d_point_cloud(new pcl::PointCloud<pcl::PointXY>);
    pcl::copyPointCloud(*pcptr_point_cloud_rgb_,*pcptr_2d_point_cloud);

    pcl::KdTreeFLANN<pcl::PointXY>::Ptr kdtree_2d_map(new pcl::KdTreeFLANN<pcl::PointXY>);
    kdtree_2d_map->setInputCloud(pcptr_2d_point_cloud);
    kdtree_z_removed_map_ = kdtree_2d_map;

    return true;
}

void SPCMapProcessing::GetPointCloudROI(double d_range_x_m, double d_range_y_m, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcptr_output)
{
    pcl::PointXYZRGB pt_search_point;
    pt_search_point.x = psstp_curr_enu_pose_.pose.position.x;
    pt_search_point.y = psstp_curr_enu_pose_.pose.position.y;
    pt_search_point.z = psstp_curr_enu_pose_.pose.position.z;

    for(auto point : pcptr_point_cloud_rgb_->points)
    {
        if(point.x > pt_search_point.x-d_range_x_m && point.x < pt_search_point.x+d_range_x_m
           && point.y > pt_search_point.y-d_range_y_m && point.y < pt_search_point.y+d_range_y_m)
        {
            pcptr_output->points.push_back(point);
        }
    }

    sensor_msgs::PointCloud2 pc2_roi_point_cloud;
    PCL2pointcloud2(*pcptr_output,pc2_roi_point_cloud);

    //============================================================ ROI_PointCloud map generatot 

    pc2_roi_point_cloud.header.frame_id = "map";
    rospub_roi_height_map_.publish(pc2_roi_point_cloud);
}

double SPCMapProcessing::GetLowestHeight(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input)
{
    if(param_DEBUG_MODE_)
    {
        std::cout<<"[MAP PUBLISHER] GetLowestHeight start"<<std::endl;
    }
    pcl::PointXY pt_search_point;

    pt_search_point.x = psstp_curr_enu_pose_.pose.position.x;
    pt_search_point.y = psstp_curr_enu_pose_.pose.position.y;

    int i_k_point_num = 100;
    std::vector<int> vec_point_idx_nearest_k_search;
    std::vector<float> vec_point_nearest_k_squared_distance;
    double d_min_height = FLT_MAX;

    if(pt_search_point.x == NAN || pt_search_point.x == INFINITY)
        return FLT_MAX;
    if(pt_search_point.y == NAN || pt_search_point.y == INFINITY)
        return FLT_MAX;
    kdtree_z_removed_map_->nearestKSearch(pt_search_point, i_k_point_num, vec_point_idx_nearest_k_search, vec_point_nearest_k_squared_distance);
    
    if(vec_point_idx_nearest_k_search.size() < 1)
    {
        if(param_DEBUG_MODE_)
        {
            ROS_WARN_STREAM("[MAP PUBLISHER] CAN NOT GET LOW HEIGHT");
        }
        return FLT_MAX;
    }

    for(int i_search_idx = 0; i_search_idx < vec_point_idx_nearest_k_search.size(); i_search_idx++)
    {
        int i_point_idx = vec_point_idx_nearest_k_search[i_search_idx];

        double d_tmp_z = pcptr_point_cloud_rgb_->points[i_point_idx].z;

        if(d_tmp_z < d_min_height)
        {
            d_min_height = d_tmp_z;
        }
    }

    if(param_DEBUG_MODE_)
        {
            std::cout<<"[MAP PUBLISHER] GetLowestHeight finish"<<std::endl;
        }

    return d_min_height;
}

double SPCMapProcessing::GetAverageHeight(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input)
{
    double d_mean_hight = 0.0;
    if(input==NULL)
    {
        ROS_ERROR_STREAM("[MAP PUBLISHER] CAN NOT GET AVERAGE HEIGHT");
        return 0;
    }

    for (unsigned int ui_point_idx = 0; ui_point_idx < input->points.size(); ui_point_idx++)
    {
        d_mean_hight += input->points[ui_point_idx].z;
    }

    d_mean_hight /= input->points.size();

    return d_mean_hight;
}

geometry_msgs::Pose SPCMapProcessing::GetMapBasedRollPitch(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input)
{
    pcl::PointXYZRGB pt_vehicle_position;
    pt_vehicle_position.x = psstp_curr_enu_pose_.pose.position.x;
    pt_vehicle_position.y = psstp_curr_enu_pose_.pose.position.y;
    pt_vehicle_position.z = psstp_curr_enu_pose_.pose.position.z;

    tf::Quaternion tfquat_heading_quat;
    geometry_msgs::Quaternion gquat_quat;
    gquat_quat = psstp_curr_enu_pose_.pose.orientation;
    tf::quaternionMsgToTF(gquat_quat,tfquat_heading_quat);
    
    tf::Matrix3x3 tfmat_quat_matrix(tfquat_heading_quat);
    double d_roll_dummy = 0.;
    double d_pitch_dummy = 0.;
    double d_heading = 0.;
    tfmat_quat_matrix.getRPY(d_roll_dummy, d_pitch_dummy, d_heading);

    pcl::PointCloud<pcl::PointXYZ>::Ptr pcptr_height_clopped_point_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    for(auto point : input->points)
    {
        if(point.z >d_height_m_-1.0 && point.z<d_height_m_+1.0)
        {
            pcl::PointXYZ pt_point;
            pt_point.x = point.x;
            pt_point.y = point.y;
            pt_point.z = point.z;

            pcptr_height_clopped_point_cloud->points.push_back(pt_point);
        }
    }

    pcl::ModelCoefficients::Ptr modcoeff_coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr ptindices_inliers (new pcl::PointIndices);

    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> sacs_sec; 
    sacs_sec.setOptimizeCoefficients (true);  // Optional
    // Mandatory
    sacs_sec.setModelType (pcl::SACMODEL_PLANE); //Use PLANE Model
    sacs_sec.setMethodType (pcl::SAC_RANSAC);  // Use RANSAC method 
    sacs_sec.setDistanceThreshold (0.01); //determines how close a point must be to the model in order to be considered an inlier
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcptr_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*pcptr_height_clopped_point_cloud,*pcptr_cloud);
    sacs_sec.setInputCloud (pcptr_cloud);
    sacs_sec.segment (*ptindices_inliers, *modcoeff_coefficients);

    double d_coeff_a = modcoeff_coefficients->values[0];
    double d_coeff_b = modcoeff_coefficients->values[1];
    double d_coeff_c = modcoeff_coefficients->values[2];
    double d_coeff_d = modcoeff_coefficients->values[3];


    // estimated plane parameters (in ax + by + cz + d = 0
    // std::cerr << "Model coefficients: " << a << " " 
    //                                     << b << " "
    //                                     << c << " " 
    //                                     << d_coeff_d << std::endl;
    
    double d_map_pitch = 0.0;
    double d_map_roll = 0.0;

    double d_check_point_x0 = pt_vehicle_position.x;
    double d_check_point_y0 = pt_vehicle_position.y;
    double d_check_point_x1 = pt_vehicle_position.x + 1.*sin(d_heading);
    double d_check_point_y1 = pt_vehicle_position.y + 1.*cos(d_heading);
    double d_check_point_x2 = pt_vehicle_position.x + 1.*cos(d_heading);
    double d_check_point_y2 = pt_vehicle_position.y + 1.*sin(d_heading);

    double d_ins_pitch_bias = -0.0232487;
    double d_pitch_diff_z = (d_coeff_a*(d_check_point_x0 - d_check_point_x1)+d_coeff_b*(d_check_point_y0 - d_check_point_y1))/d_coeff_c; // z1 - z0
    double d_pitch_diff_length = 1.0;
    double d_pitch_vehicle = atan2(d_pitch_diff_z,d_pitch_diff_length)+d_ins_pitch_bias;

    double d_ins_roll_bias = 0.05;
    double d_roll_diff_z = (d_coeff_a*(d_check_point_x0 - d_check_point_x2)+d_coeff_b*(d_check_point_y0 - d_check_point_y2))/d_coeff_c; // z2 - z0
    double d_roll_diff_length = 1.0;
    double d_roll_vehicle = atan2(d_roll_diff_z,d_roll_diff_length) + d_ins_roll_bias;

    double d_vehicle_height_z = -(d_coeff_a*pt_vehicle_position.x + d_coeff_b* pt_vehicle_position.y + d_coeff_d)/d_coeff_c;


    if(d_pitch_diff_length < 0.1)
    {
        d_pitch_vehicle = 0.0;
    }
    if(d_roll_diff_length < 0.1)
    {
        d_roll_vehicle = 0.0;
    }

    d_map_pitch = d_pitch_vehicle;
    d_map_roll  = d_roll_vehicle;

    tf::Quaternion tfquat_result_quat;
    tfquat_result_quat.setRPY(-d_map_roll,-d_map_pitch,d_heading);
    geometry_msgs::Quaternion gquat_result_quat_msg;

    tf::quaternionTFToMsg(tfquat_result_quat,gquat_result_quat_msg);
    geometry_msgs::Pose pose_output_msg;
    pose_output_msg.orientation = gquat_result_quat_msg;
    pose_output_msg.position.z = d_vehicle_height_z;
    return pose_output_msg;
}
void SPCMapProcessing::PointCloudPublisher() {
    if(param_DEBUG_MODE_)
    {
        std::cout<<"[MAP PUBLISHER] PointCloudPublisher start"<<std::endl;
    }
    rospub_map_rgb_.publish(pc2_point_cloud_rgb_);
    if(param_DEBUG_MODE_)
    {
        std::cout<<"[MAP PUBLISHER] PointCloudPublisher finish"<<std::endl;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "spc_map_processing");

    SPCMapProcessing SPCMapProcessing;

    bool b_is_global_map_published = false;

    // Loop Rate
    ros::Rate rosrate_loop_freq(10);
    int i_count = 0;

    while (ros::ok()) {
        // if(i_count % 100 == 0)
        if(!b_is_global_map_published)
        {
            SPCMapProcessing.PointCloudPublisher();
            b_is_global_map_published = true;
        }
        
        i_count++;

        ros::spinOnce();
        rosrate_loop_freq.sleep();
    }
    return 0;
}