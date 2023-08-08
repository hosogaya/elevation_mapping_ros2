#pragma once

#include <rclcpp/rclcpp.hpp>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_cv/grid_map_cv.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>

#include <opencv2/imgproc/imgproc_c.h>

namespace plane_extraction{

class PlaneExtractor : public rclcpp::Node
{
public:
    PlaneExtractor(const rclcpp::NodeOptions option);
    ~PlaneExtractor();

private:
    rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr sub_grid_map_;
    void callbackGridMap(const grid_map_msgs::msg::GridMap::UniquePtr _source_map);

    rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr pub_grid_map_;
    void publishGridMap(const grid_map::GridMap& _src);

    bool extractOperatingRange(const grid_map::GridMap& _src, grid_map::GridMap& _dst);
    bool divideByTraversability(grid_map::GridMap& _src);
    bool divideByNormalVector(grid_map::GridMap& _src);
    bool dividePlane(grid_map::GridMap& _src, const std::string& input_layer, const std::string& output_layer);

    void addData(const double& data, int& size, double& mean, double& variance);
    void removeData(const double& data, int& size, double& mean, double& variance);

    struct Cell
    {
        Cell(const grid_map::Index& ind, const double& v) : index(ind), value(v) {}
        double value;
        grid_map::Index index;
    };
    
};

}