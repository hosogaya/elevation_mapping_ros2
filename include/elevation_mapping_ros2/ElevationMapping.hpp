/**
 * @brief
 * Manage all process. 
*/

#include <rclcpp/rclcpp.hpp>
#include <grid_map_core.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <elevation_mapping/ElevationMap.hpp>

namespace elevation_mapping
{
class ElevationMapping : public rclcpp::Node
{
public:
    ElevationMapping();
    ~ElevationMapping();

private:
    void callbackPointCloud(const sensor_msgs::msg::PointCloud2::UniquePtr _point_cloud, const geometry_msgs::msg::PoseWithCovarianceStamped _pose);
    
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub_point_cloud_;
    message_filters::Subscriber<geometry_msgs::msg::PoseWithCovarianceStamped> sub_pose_;

    message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime
        <sensor_msgs::msg::PointCloud2, geometry_msgs::msg::PoseWithCovarianceStamped>> sync_sub_;
        
};

}