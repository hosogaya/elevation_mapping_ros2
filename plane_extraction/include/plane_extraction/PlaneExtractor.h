#pragma once

#include <rclcpp/rclcpp.hpp>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_cv/grid_map_cv.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>

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
    bool divideByTraversability(const grid_map::GridMap& _src, grid_map::GridMap& _dst);
    bool extractWithNormalVector(const grid_map::GridMap& _src, grid_map::GridMap& _dst);

    struct Cell
    {
        Cell(const grid_map::Index& ind, const double& v) : index(ind), value(v) {}
        double value;
        grid_map::Index index;
    };
    
};

}