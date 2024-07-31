/**
 * @brief sensor pre-proccessing
 * compute height and height variance. 
*/
#pragma once

#include <memory>
#include <functional>

#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/point_representation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>

#include <elevation_mapping_ros2/TypeDef.hpp>

#include <rclcpp/rclcpp.hpp>

#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
// #include <tf2_helpers/tf2_conversion_helpers.hpp>

namespace elevation_mapping
{
class SensorProcessorBase
{
public:
    struct CommonConfig
    {
        std::string sensor_frame_ = "lidar";
        // std::string point_cloud_topic_name_ = "/point_cloud";

        std::string logger_name_ = "SensorProcessor";
        bool use_voxel_filter_ = false;
        double voxel_leaf_size_ = 0.05; // mm
        double pass_filter_lower_threshold_ = std::numeric_limits<double>::min();
        double pass_filter_upper_threshold_ = std::numeric_limits<double>::max();
    };
    SensorProcessorBase(CommonConfig config, std::string map_frame, std::string robot_frame);
    ~SensorProcessorBase();

    bool process(const sensor_msgs::msg::PointCloud2& _point_cloud, const Eigen::Matrix<double, 6, 6>& _robot_covariance , PointCloudType::Ptr& _processed_point_cloud_map_frame, Eigen::VectorXf& _variance);
    bool isFirstTfAvailable() const {return first_tf_available_;}
    const Eigen::Affine3d& getTransformSensor2Map() {return transform_sensor2map_;}
protected:
    bool updateTransformations();
    bool transformPointCloud(const PointCloudType& _point_cloud, PointCloudType::Ptr& _trnasformed_point_cloud, const std::string& _target_frame);

    bool removeNans(PointCloudType::Ptr _point_cloud);
    bool reducePoint(PointCloudType::Ptr _point_cloud);
    bool removeOutsideLimits(const PointCloudType::Ptr& _reference, std::vector<PointCloudType::Ptr>& _point_clouds);
    virtual bool filterSensorType(PointCloudType::Ptr _point_cloud) = 0;

    virtual void computeVariance(const PointCloudType::Ptr _point_cloud, const Eigen::Matrix<double, 6, 6>& _robot_covariance, Eigen::VectorXf& _variance) = 0;

    Eigen::Matrix3f computeSkewMatrixfromVector(const Eigen::Vector3f& _vec);

    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    rclcpp::Clock::SharedPtr clock_;
    // rclcpp::Logger logger_;

    const std::string kSensorFrameID_;
    const std::string kRobotFrameID_;
    const std::string kMapFrameID_;

    // transform
    Eigen::Affine3d transform_sensor2map_;
    Eigen::Matrix3d rotation_base2sensor_;
    Eigen::Vector3d translation_base2sensor_; // in base frame
    Eigen::Matrix3d rotation_map2base_;
    Eigen::Vector3d translation_map2base_; // in map frame

    // state
    bool first_tf_available_ = false;
    tf2::TimePoint current_time_point_;

    // option
    bool use_voxel_filter_ = false;
    double voxel_leaf_size_ = 20.0; // mm

    double pass_throught_lower_threshold_ = -std::numeric_limits<double>::infinity(); // relative to robot frame
    double pass_throught_upper_threshold_ = std::numeric_limits<double>::infinity();

    const std::string logger_name_ = "SensorProcessor";
};
}