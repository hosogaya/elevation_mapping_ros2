#include <rclcpp/rclcpp.hpp>
#include <elevation_mapping_ros2/post_processing/PostProcessor.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options = rclcpp::NodeOptions().use_intra_process_comms(true);
    rclcpp::spin(std::make_shared<elevation_mapping::PostProcessor>(options));
    rclcpp::shutdown();

    return 0;
}