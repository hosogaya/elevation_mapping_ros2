#pragma once
#include <elevation_mapping_ros2/sesnor_processing/SensorProcessorBase.hpp>

namespace elevation_mapping
{

class LaserSensorProcessor: public SensorProcessorBase
{
public:
    struct LaserSensorConfig
    {
        double min_radius_ = 0.02;
        double beam_angle_ = 0.0006;
        double beam_constant_ = 0.0015;
    };

    LaserSensorProcessor(CommonConfig common_config, LaserSensorConfig laser_config, std::string map_frame, std::string robot_frame);
    ~LaserSensorProcessor();

private:
    bool filterSensorType(PointCloudType::Ptr _point_cloud) override;
    void computeVariance(const PointCloudType::Ptr _point_cloud, const Eigen::Matrix<double, 6, 6>& _robot_covariance, Eigen::VectorXf& _variance) override;

    double min_radius_square_;
    double beam_angle_;
    double beam_constant_;
};
}