#include <elevation_mapping_ros2/sesnor_processing/PerfectSensorProcessor.hpp>


namespace elevation_mapping
{
PerfectSensorProcessor::PerfectSensorProcessor(
    const std::string _sensor_frame, const std::string _map_frame, 
    const rclcpp::Logger _logger)
    : SensorProcessorBase(_sensor_frame, _map_frame, _logger)
{

}

PerfectSensorProcessor::~PerfectSensorProcessor() {}

bool PerfectSensorProcessor::computeVariance(PointCloudType::Ptr _point_cloud, const Eigen::Matrix<double, 6, 6>& _robot_covariance, Eigen::VectorXf& _variance)
{
    _variance.resize(_point_cloud->size());

    // projection vector
    const Eigen::RowVector3f projection_vector = Eigen::RowVector3f::UnitZ();

    // sensor to map jacobian
    const Eigen::RowVector3f sensor_jac = projection_vector*(rotation_map2base_.transpose()*rotation_base2sensor_.transpose()).cast<float>();

    // Robot rotation covariance
    const Eigen::Matrix3f rotation_variance = _robot_covariance.bottomRightCorner(3, 3).cast<float>();

    // Preparations for robot rotation jacobian
    const Eigen::Matrix3f R_B2M_transpose = rotation_map2base_.transpose().cast<float>();
    const Eigen::RowVector3f Pro_R_B2M_transpose = projection_vector*R_B2M_transpose;
    const Eigen::Matrix3f R_S2B_transpose = rotation_base2sensor_.transpose().cast<float>();
    const Eigen::Matrix3f T_B2S_skew = computeSkewMatrixfromVector(translation_base2sensor_.cast<float>());

    for (size_t i=0; i<_point_cloud->size(); ++i)
    {
        const auto point{_point_cloud->points[i]};
        Eigen::Vector3f point_vector{point.x, point.y, point.z};
        float height_variance = 0.0;

        // compute sensor covariance matirx
        float variance_normal = 0.0;
        float variance_lateral = 0.0;
        Eigen::Matrix3f sensor_variance = Eigen::Matrix3f::Zero();
        sensor_variance.diagonal() << variance_lateral, variance_lateral, variance_normal;
        
        // robot rotation jacobian
        const Eigen::Matrix3f T_S2P_inB_skew = computeSkewMatrixfromVector(R_S2B_transpose*point_vector); // R_S2B_transpose_times_T_B2S_skew
        const Eigen::RowVector3f rotation_jac = Pro_R_B2M_transpose*(T_S2P_inB_skew+T_B2S_skew); // jacobian  d{(projection)*(B2P in map frame)}/d{rotation angle} 

        // variance for map 
        height_variance = rotation_jac*rotation_variance*rotation_jac.transpose();
        height_variance += sensor_jac*sensor_variance*sensor_jac.transpose();

        _variance(i) = height_variance;
    }
}

void PerfectSensorProcessor::readParameters(rclcpp::Node* _node)
{
    _node->declare_parameter("use_voxel_filter", param_voxel_grid_fitler_.use_filter, true);
    _node->declare_parameter("voxel_leaf_size", param_voxel_grid_fitler_.leaf_size, 5.0);
    _node->declare_parameter("pass_filter_lower_threshold", param_pass_through_filter_.lower_threshold_, -std::numeric_limits<double>::infinity());
    _node->declare_parameter("pass_filter_upper_threshold", param_pass_through_filter_.upper_threshold_, std::numeric_limits<double>::infinity());
}

}