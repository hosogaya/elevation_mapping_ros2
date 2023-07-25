#include <elevation_mapping_ros2/sesnor_processing/SensorProcessorBase.hpp>

namespace elevation_mapping
{

SensorProcessorBase::SensorProcessorBase(
    const std::string _sensor_frame, const std::string _map_frame, 
    const rclcpp::Logger _logger)
    : kSensorFrameID_(_sensor_frame), kMapFrameID_(_map_frame)
{
    clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    logger_ = _logger;
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(clock_);
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);   
}

SensorProcessorBase::~SensorProcessorBase() {}

bool SensorProcessorBase::process(const PointCloudType& _point_cloud, const Eigen::Matrix<double, 6, 6>& _robot_covariance,  PointCloudType::Ptr& _processed_point_cloud_map_frame, Eigen::VectorXf& _variance)
{
    /** listening transform from sensor frame to map frame*/
    rclcpp::Time time_stamp(_point_cloud.header.stamp*1000); // mciro-sec to nano-sec
    if (!updateTransformations(time_stamp)) return false;

    // transform into sensor frame
    PointCloudType::Ptr point_cloud_sensor_frame;
    transformPointCloud(_point_cloud, point_cloud_sensor_frame, kSensorFrameID_);

    // remove Nans 
    removeNans(point_cloud_sensor_frame);

    // reduce points using voxel grid filter
    reducePoint(point_cloud_sensor_frame);

    // specific filtering per sensor type
    filterSensorType(point_cloud_sensor_frame);

    // transform into map frame
    if (!transformPointCloud(*point_cloud_sensor_frame, _processed_point_cloud_map_frame, kMapFrameID_))
    {
        return false;
    }

    // remove outside limits in map frame
    std::vector<PointCloudType::Ptr> point_clouds{point_cloud_sensor_frame, _processed_point_cloud_map_frame};
    removeOutsideLimits(_processed_point_cloud_map_frame, point_clouds);

    // compute variance 
    computeVariance(point_cloud_sensor_frame, _robot_covariance, _variance);
}

bool SensorProcessorBase::updateTransformations(const rclcpp::Time& _time_stamp)
{
    try {
        // sensor to map
        geometry_msgs::msg::TransformStamped transformTF = tf_buffer_->lookupTransform(kSensorFrameID_, kMapFrameID_, _time_stamp, rclcpp::Duration(1.0));
        tf2::fromMsg(transformTF, transform_sensor2map_);
        
        // base to sensor
        transformTF = tf_buffer_->lookupTransform(kBaseLinkID_, kSensorFrameID_, _time_stamp, rclcpp::Duration(1.0));
        Eigen::Affine3d transform;
        tf2::fromMsg(transformTF, transform);
        rotation_base2sensor_ = transform.rotation().matrix();
        translation_base2sensor_ = transform.translation();
        
        // map to base
        transformTF = tf_buffer_->lookupTransform(kMapFrameID_, kBaseLinkID_, _time_stamp, rclcpp::Duration(1.0));
        tf2::fromMsg(transformTF, transform);
        rotation_map2base_ = transform.rotation().matrix();
        translation_map2base_ = transform.translation();

        if (!first_tf_available_) first_tf_available_ = true;
        return true;
    }
    catch(const std::exception& e)
    {
        if (!first_tf_available_) return false;
        std::cerr << e.what() << '\n';
        return false;
    }
    
}

bool SensorProcessorBase::transformPointCloud(const PointCloudType& _point_cloud, PointCloudType::Ptr& _trnasformed_point_cloud, const std::string& _target_frame)
{
    rclcpp::Time time_stamp(_point_cloud.header.stamp*1000); // mciro-sec to nano-sec
    const std::string& input_frame = _point_cloud.header.frame_id;
    geometry_msgs::msg::TransformStamped transformTF;
    try {
        transformTF = tf_buffer_->lookupTransform(_target_frame, input_frame, time_stamp, rclcpp::Duration(1.0));
    }
    catch (tf2::TransformException& ex) {
        RCLCPP_ERROR(logger_, "%s", ex.what());
        return false;
    }
    Eigen::Affine3d transform;
    tf2::fromMsg(transformTF, transform);
    pcl::transformPointCloud(_point_cloud, *_trnasformed_point_cloud, transform.cast<float>(), true);
    

    return true;
}

bool SensorProcessorBase::removeNans(PointCloudType::Ptr _point_cloud)
{
    pcl::Indices indices;
    PointCloudType temp_point_cloud;
    if (!_point_cloud->is_dense)
    {
        pcl::removeNaNFromPointCloud(*_point_cloud, temp_point_cloud, indices);
        _point_cloud->points.swap(temp_point_cloud.points);
        return true;
    }
    return false;
}

bool SensorProcessorBase::reducePoint(PointCloudType::Ptr _point_cloud)
{
    if (param_voxel_grid_fitler_.use_filter)
    {
        PointCloudType temp_point_cloud;
        pcl::VoxelGrid<PointType> voxel_grid_filter;
        voxel_grid_filter.setInputCloud(_point_cloud);
        voxel_grid_filter.setLeafSize(param_voxel_grid_fitler_.leaf_size, param_voxel_grid_fitler_.leaf_size, param_voxel_grid_fitler_.leaf_size);
        voxel_grid_filter.filter(temp_point_cloud);
        _point_cloud->points.swap(temp_point_cloud.points);
        return true;
    }
    else return true;
}

bool SensorProcessorBase::removeOutsideLimits(const PointCloudType::Ptr& _reference, std::vector<PointCloudType::Ptr>& _point_clouds)
{
    if (!std::isfinite(param_pass_through_filter_.lower_threshold_) && !std::isfinite(param_pass_through_filter_.upper_threshold_))
    {
        RCLCPP_DEBUG(logger_, "pass through filter is not applied");
        return true;
    }
    RCLCPP_DEBUG(logger_, "Limiting point cloud to the height interval of [%lf, %lf] relative to the robot base", param_pass_through_filter_.lower_threshold_, param_pass_through_filter_.upper_threshold_);

    pcl::PassThrough<PointType> pass_through_filter(true);
    pass_through_filter.setInputCloud(_reference);
    pass_through_filter.setFilterFieldName("z");
    double lower = translation_map2base_.z() + param_pass_through_filter_.lower_threshold_;
    double upper = translation_map2base_.z() + param_pass_through_filter_.upper_threshold_;
    pass_through_filter.setFilterLimits(lower, upper);
    pcl::IndicesPtr inside_indeces(new std::vector<int>);
    pass_through_filter.filter(*inside_indeces);

    for (auto& point_cloud: _point_clouds)
    {
        pcl::ExtractIndices<PointType> extract_indices_filter;
        extract_indices_filter.setInputCloud(point_cloud);
        extract_indices_filter.setIndices(inside_indeces);
        PointCloudType temp_point_cloud;
        extract_indices_filter.filter(temp_point_cloud);
        point_cloud->points.swap(temp_point_cloud.points);
    }
    RCLCPP_DEBUG(logger_, "Remove point out side limits. Reduced point cloud to %ld points", (_point_clouds[0]->size()));
    return true;
}

Eigen::Matrix3f SensorProcessorBase::computeSkewMatrixfromVector(const Eigen::Vector3f& _vec)
{
    Eigen::Matrix3f mat;
    mat.setZero();
    mat(0, 1) =-_vec.z();
    mat(0, 2) = _vec.y();
    mat(1, 0) = _vec.z();
    mat(1, 2) =-_vec.x();
    mat(2, 0) =-_vec.y();
    mat(2, 1) = _vec.z();
}


}

