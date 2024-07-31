#include <elevation_mapping_ros2/sesnor_processing/PerfectSensorProcessor.hpp>


namespace elevation_mapping
{
PerfectSensorProcessor::PerfectSensorProcessor(CommonConfig config, std::string map_frame, std::string robot_frame)    
: SensorProcessorBase(config, map_frame, robot_frame)
{}

PerfectSensorProcessor::~PerfectSensorProcessor() {}

bool PerfectSensorProcessor::filterSensorType(PointCloudType::Ptr _point_cloud)
{
    return true;
}

void PerfectSensorProcessor::computeVariance(const PointCloudType::Ptr _point_cloud, const Eigen::Matrix<double, 6, 6>& _robot_covariance, Eigen::VectorXf& _variance)
{
    _variance.resize(_point_cloud->size());

    // projection vector
    const Eigen::RowVector3f projection_vector = Eigen::RowVector3f::UnitZ();

    // sensor to map jacobian (unnecessary)
    // const Eigen::RowVector3f sensor_jac = projection_vector*(rotation_map2base_.transpose()*rotation_base2sensor_.transpose()).cast<float>();

    // Robot rotation covariance
    const Eigen::Matrix3f rotation_variance = _robot_covariance.bottomRightCorner(3, 3).cast<float>();

    // Preparations for robot rotation jacobian
    const Eigen::Matrix3f rotation_map2base_transpose = rotation_map2base_.transpose().cast<float>();
    const Eigen::RowVector3f projected_rotation_map2base_transpose = projection_vector*rotation_map2base_transpose;
    const Eigen::Matrix3f rotation_base2sensor_transpose = rotation_base2sensor_.transpose().cast<float>();
    const Eigen::Matrix3f translation_base2sensor_skew = computeSkewMatrixfromVector(translation_base2sensor_.cast<float>());

    for (size_t i=0; i<_point_cloud->size(); ++i)
    {
        const auto point{_point_cloud->points[i]};
        Eigen::Vector3f point_vector{point.x, point.y, point.z};
        float height_variance = 0.0;

        float measurement_distance2 = std::pow(point.x, 2.0)+std::pow(point.y, 2.0) + std::pow(point.z, 2.0);

        // sensor covariance is always zero
        // Eigen::Matrix3f sensor_variance = Eigen::Matrix3f::Zero();
        
        // robot rotation jacobian
        const Eigen::Matrix3f translation_sensor2point_skew_in_base_frame = computeSkewMatrixfromVector(rotation_base2sensor_transpose*point_vector); 
        const Eigen::RowVector3f rotation_jac = projected_rotation_map2base_transpose*(translation_base2sensor_skew + translation_sensor2point_skew_in_base_frame); 

        // variance for map 
        height_variance = rotation_jac*rotation_variance*rotation_jac.transpose();
        // height_variance += sensor_jac*sensor_variance*sensor_jac.transpose(); // zero

        _variance(i) = height_variance;
        assert( _variance(i) >= 0.0 && "Variance of point cloud is lower than 0");
    }
}

}