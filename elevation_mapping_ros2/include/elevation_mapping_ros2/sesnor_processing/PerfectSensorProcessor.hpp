#pragma once

#include <elevation_mapping_ros2/sesnor_processing/SensorProcessorBase.hpp>

namespace elevation_mapping
{

class PerfectSensorProcessor : public SensorProcessorBase
{
public: 
    explicit PerfectSensorProcessor(CommonConfig config, std::string map_frame, std::string robot_frame);

    ~PerfectSensorProcessor();
private:
    bool filterSensorType(PointCloudType::Ptr _point_cloud) override;
    void computeVariance(const PointCloudType::Ptr _point_cloud, const Eigen::Matrix<double, 6, 6>& _robot_covariance, Eigen::VectorXf& _variance) override;
};  

}