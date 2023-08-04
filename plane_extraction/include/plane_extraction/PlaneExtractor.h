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
    PlaneExtractor();
    ~PlaneExtractor();

private:
    rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr sub_grid_map_;
    void callbackGridMap(const grid_map_msgs::msg::GridMap::UniquePtr _source_map);

    bool extractWithGeometry(const grid_map::GridMap& _src, grid_map::GridMap& _dst);
    bool extractWithTraversability(const grid_map::GridMap& _src, std::vector<grid_map::GridMap>& _dst);
    bool extractWithNormalVector(const grid_map::GridMap& _src, std::vector<grid_map::GridMap>& _dst);

    typedef struct Cell
    {
        double value;
        grid_map::Index index;
    };
    
};

}