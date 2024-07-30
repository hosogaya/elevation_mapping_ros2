#include <elevation_mapping_ros2/sesnor_processing/LaserSensorProcessor.hpp>

namespace elevation_mapping
{
LaserSensorProcessor::LaserSensorProcessor(const std::string& _sensor_frame, const std::string& _map_frame, const std::string& _robot_frame)
:  SensorProcessorBase(_sensor_frame, _map_frame, _robot_frame)
{}

LaserSensorProcessor::~LaserSensorProcessor() {}


bool LaserSensorProcessor::filterSensorType(PointCloudType::Ptr _point_cloud)
{
    return true;
}

void LaserSensorProcessor::computeVariance(const PointCloudType::Ptr _point_cloud, const Eigen::Matrix<double, 6, 6>& _robot_covariance, Eigen::VectorXf& _variance)
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

    const float& variance_normal = min_radius_square_;

    for (size_t i=0; i<_point_cloud->size(); ++i)
    {
        const auto& point{_point_cloud->points[i]};
        Eigen::Vector3f point_vector{point.x, point.y, point.z};

        const float measurement_distance2 = point.x*point.x + point.y*point.y + point.z*point.z;

        // compute sensor covariance matirx
        float variance_lateral = beam_constant_ + beam_angle_*measurement_distance2;
        variance_lateral += variance_lateral;

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

void LaserSensorProcessor::readParameters(rclcpp::Node* _node)
{
    param_voxel_grid_fitler_.use_filter = _node->declare_parameter("sensor.use_voxel_filter", true);
    param_voxel_grid_fitler_.leaf_size = _node->declare_parameter("sensor.voxel_leaf_size", 5.0);
    param_pass_through_filter_.lower_threshold_ = _node->declare_parameter("sensor.pass_filter_lower_threshold", -std::numeric_limits<double>::infinity());
    param_pass_through_filter_.upper_threshold_ = _node->declare_parameter("sensor.pass_filter_upper_threshold", std::numeric_limits<double>::infinity());
    logger_name_ = _node->declare_parameter("sensor.logger_name", "LaserSensorProcessor");

    min_radius_square_ = _node->declare_parameter("sensor.laser_min_radius", 0.02);
    min_radius_square_ *= min_radius_square_;
    beam_angle_ = _node->declare_parameter("sensor.laser_beam_angle", 0.0006);
    beam_constant_ = _node->declare_parameter("sensor.laser_beam_constant", 0.0015); 
}

}