#include <elevation_mapping_ros2/sesnor_processing/SensorProcessorBase.hpp>

namespace elevation_mapping
{

class LaserSensorProcessor: public SensorProcessorBase
{
public:
    LaserSensorProcessor(const std::string& _sensor_frame, const std::string& _map_frame, const std::string& _robot_frame);
    ~LaserSensorProcessor();

private:
    virtual void computeVariance(const PointCloudType::Ptr _point_cloud, const Eigen::Matrix<double, 6, 6>& _robot_covariance, Eigen::VectorXf& _variance) override;

    double min_radius_square_;
    double beam_angle_;
    double beam_constant_;
};
}