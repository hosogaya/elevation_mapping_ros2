#pragma once

#include <elevation_mapping_ros2/sesnor_processing/SensorProcessorBase.hpp>

namespace elevation_mapping
{

class PerfectSensorProcessor : public SensorProcessorBase
{
public: 
    explicit PerfectSensorProcessor(const std::string sensor_frame, const std::string map_frame, 
                const rclcpp::Logger _logger);

    ~PerfectSensorProcessor();
    void readParameters(rclcpp::Node* _node) override;
private:
    bool computeVariance(PointCloudType::Ptr _point_cloud, const Eigen::Matrix<double, 6, 6>& _robot_covariance, Eigen::VectorXf& _variance) override;
};  

}