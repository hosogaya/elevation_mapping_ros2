#pragma once

#include <elevation_mapping_ros2/sesnor_processing/SensorProcessorBase.hpp>

namespace elevation_mapping
{
class StereoSensorProcessor: public SensorProcessorBase
{
public:
    struct StereoSensorConfig
    {
        double depth_upper_limit_ = 20.0;
        double depth_lower_limit_ = 0.1;
        double coef_normal_variance_ = 0.0001;
        double coef_lateral_variance_ = 0.0001;
    };

    StereoSensorProcessor(CommonConfig common_config, StereoSensorConfig stereo_config, std::string map_frame, std::string robot_frame);
    ~StereoSensorProcessor();

private:
    bool filterSensorType(PointCloudType::Ptr _point_cloud) override;
    void computeVariance(const PointCloudType::Ptr _point_cloud, const Eigen::Matrix<double, 6, 6>& _robot_covariance, Eigen::VectorXf& _variance) override;

    double depth_upper_limit_;
    double depth_lower_limit_;
    double coef_normal_variance_;
    double coef_lateral_variance_;
};
}