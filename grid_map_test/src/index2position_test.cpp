#include <rclcpp/rclcpp.hpp>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/grid_map_ros.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("test_node");

    grid_map::GridMap grid_map;
    grid_map::Length length{10.0, 10.0};
    grid_map::Position position{0.0, 0.0};
    const double resolution = 0.05;
    grid_map.setGeometry(length, resolution, position);

    grid_map::Position move_position{2.0, 6.0};
    grid_map.move(move_position);

    grid_map::Index top_left{0, 0};
    grid_map::Index bottom_right{199, 199};
    grid_map::Index top_right{0, 199};
    grid_map::Index bottom_left{199, 0};

    grid_map::Position temp_pos;
    grid_map.getPosition(top_left, temp_pos);
    RCLCPP_INFO(node->get_logger(), "Index: (%d, %d), pos: (%f, %f)", top_left.x(), top_left.y(), temp_pos.x(), temp_pos.y());

    grid_map.getPosition(top_right, temp_pos);
    RCLCPP_INFO(node->get_logger(), "Index: (%d, %d), pos: (%f, %f)", top_right.x(), top_right.y(), temp_pos.x(), temp_pos.y());

    grid_map.getPosition(bottom_left, temp_pos);
    RCLCPP_INFO(node->get_logger(), "Index: (%d, %d), pos: (%f, %f)", bottom_left.x(), bottom_left.y(), temp_pos.x(), temp_pos.y());

    grid_map.getPosition(bottom_right, temp_pos);
    RCLCPP_INFO(node->get_logger(), "Index: (%d, %d), pos: (%f, %f)", bottom_right.x(), bottom_right.y(), temp_pos.x(), temp_pos.y());

    rclcpp::shutdown();

    return 0;
}