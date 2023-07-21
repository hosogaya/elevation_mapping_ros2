/**
 * @brief manage map data. 
 * raw_map_: not fused map. update this map.
 * fused_map_: raw_maps are fused to this map. 
*/

#include <rclcpp/rclcpp.hpp>
#include <grid_map_core.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

namespace elevation_mapping
{
class ElevationMap: public rclcpp::Node
{
public:
    ElevationMap();
    ~ElevationMap();

private:
    bool setParameters();

    std::unique_ptr<grid_map::GridMap> raw_map_;
    std::unique_ptr<grid_map::GridMap> fused_map_;

};

}