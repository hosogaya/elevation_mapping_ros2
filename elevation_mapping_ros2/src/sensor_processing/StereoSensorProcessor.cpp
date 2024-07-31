#include <elevation_mapping_ros2/sesnor_processing/StereoSensorProcessor.hpp>
// #include <elevation_mapping_ros2/sensor_processing/SensorProcessorBase.hpp>

namespace elevation_mapping
{
StereoSensorProcessor::StereoSensorProcessor(CommonConfig common_config, StereoSensorConfig stereo_config, std::string map_frame, std::string robot_frame)
    : SensorProcessorBase(common_config, map_frame, robot_frame)
{
    depth_lower_limit_ = stereo_config.depth_lower_limit_;
    depth_upper_limit_ = stereo_config.depth_upper_limit_;
    coef_lateral_variance_ = stereo_config.coef_lateral_variance_;
    coef_normal_variance_ = stereo_config.coef_normal_variance_;
}

StereoSensorProcessor::~StereoSensorProcessor() {}

bool StereoSensorProcessor::filterSensorType(PointCloudType::Ptr _point_cloud)
{
    pcl::PassThrough<PointType> pass_through_filter(true);
    pass_through_filter.setInputCloud(_point_cloud);
    pass_through_filter.setFilterFieldName("z");
    pass_through_filter.setFilterLimits(depth_lower_limit_, depth_upper_limit_);

    PointCloudType temp_point_cloud;
    pass_through_filter.filter(temp_point_cloud);
    _point_cloud->swap(temp_point_cloud);

    return true;
}

void StereoSensorProcessor::computeVariance(const PointCloudType::Ptr _point_cloud, const Eigen::Matrix<double, 6, 6>& _robot_covariance, Eigen::VectorXf& _variance)
{
    _variance.resize(_point_cloud->size());

    // projection vector
    const Eigen::RowVector3f projection_vector = Eigen::RowVector3f::UnitZ();

    // sensor to map jacobian
    const Eigen::RowVector3f sensor_jac = projection_vector*(rotation_map2base_.transpose()*rotation_base2sensor_.transpose()).cast<float>();

    // Robot rotation covariance
    const Eigen::Matrix3f rotation_variance = _robot_covariance.bottomRightCorner(3, 3).cast<float>();

    // Preparations for robot rotation jacobian
    const Eigen::Matrix3f rotation_map2base_transpose = rotation_map2base_.transpose().cast<float>();
    const Eigen::RowVector3f projected_rotation_map2base_transpose = projection_vector*rotation_map2base_transpose;
    const Eigen::Matrix3f rotation_base2sensor_transpose = rotation_base2sensor_.transpose().cast<float>();
    const Eigen::Matrix3f translation_base2sensor_skew = computeSkewMatrixfromVector(translation_base2sensor_.cast<float>());

    for (size_t i=0; i<_point_cloud->size(); ++i)
    {
        const auto& point{_point_cloud->points[i]};
        Eigen::Vector3f point_vector{point.x, point.y, point.z};

        float measurement_distance2 = point.x*point.x + point.y*point.y + point.z*point.z;

        // compute sensor covariance matirx
        float variance_normal = coef_normal_variance_* measurement_distance2;
        float variance_lateral = coef_lateral_variance_*measurement_distance2;
        Eigen::Matrix3f sensor_variance = Eigen::Matrix3f::Zero();
        sensor_variance.diagonal() << variance_lateral, variance_lateral, variance_normal;
        
        // robot rotation jacobian
        const Eigen::Matrix3f translation_sensor2point_skew_in_base_frame = computeSkewMatrixfromVector(rotation_base2sensor_transpose*point_vector); 
        const Eigen::RowVector3f rotation_jac = projected_rotation_map2base_transpose*(translation_base2sensor_skew + translation_sensor2point_skew_in_base_frame); 

        // variance for map 
        float height_variance = rotation_jac*rotation_variance*rotation_jac.transpose();
        height_variance += sensor_jac*sensor_variance*sensor_jac.transpose();

        _variance(i) = height_variance;
        assert( _variance(i) >= 0.0 && "Variance of point cloud is lower than 0");
    }
}

}