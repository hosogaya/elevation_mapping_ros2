#include <plane_extraction/PlaneExtractor.h>

namespace plane_extraction
{
using namespace std::placeholders;
PlaneExtractor::PlaneExtractor(const rclcpp::NodeOptions options) : rclcpp::Node("plane_extractor", options)
{
    pub_grid_map_ = create_publisher<grid_map_msgs::msg::GridMap>(
        "output/grid_map", 10
    );

    sub_grid_map_ = create_subscription<grid_map_msgs::msg::GridMap>(
        "input/grid_map", 10, std::bind(&PlaneExtractor::callbackGridMap, this, _1)
    );
}

PlaneExtractor::~PlaneExtractor() {}

void PlaneExtractor::callbackGridMap(const grid_map_msgs::msg::GridMap::UniquePtr _msg)
{
    RCLCPP_INFO(get_logger(), "Convert message to grid_map");
    grid_map::GridMap src_map;
    grid_map::GridMapRosConverter::fromMessage(*_msg, src_map);

    RCLCPP_INFO(get_logger(), "Extract map in operating range");
    grid_map::GridMap sub_map;
    if (!extractOperatingRange(src_map, sub_map))
    {
        RCLCPP_ERROR(get_logger(), "Failed to extract map in operating range");
        return;
    }
    RCLCPP_INFO(get_logger(), "Divids maps according to traversability");
    grid_map::GridMap traversabilty_map;
    if (!divideByTraversability(sub_map, traversabilty_map))
    {
        RCLCPP_ERROR(get_logger(), "Failed to divids maps according to traversability");
        return;
    }

    dividePlane(traversabilty_map, "travesability0", "plane");
    RCLCPP_INFO(get_logger(), "Publish grid_map");
    publishGridMap(traversabilty_map);
    
}

bool PlaneExtractor::extractOperatingRange(const grid_map::GridMap& _src, grid_map::GridMap& _dst)
{
    //@TODO How to deremine center and length
    grid_map::Position center = _src.getPosition();
    grid_map::Length length{1.0, 1.0};
    bool is_success = false;
    _dst = _src.getSubmap(center, length, is_success);

    return is_success;
}

bool PlaneExtractor::divideByTraversability(const grid_map::GridMap& _src, grid_map::GridMap& _dst)
{
    RCLCPP_INFO(get_logger(), "Sort grid map index according to traversability");
    std::vector<Cell> cells;
    for (grid_map::GridMapIterator iterator(_src); !iterator.isPastEnd(); ++iterator)
    {
        double value = _src.at("traversability", *iterator);
        if (std::isnan(value)) continue;
        cells.emplace_back(*iterator, value);
    }
    std::sort(cells.begin(), cells.end(), [](const auto& x, const auto& y){return x.value > y.value;});
    RCLCPP_INFO(get_logger(), "Number of cells not Nan: %d", cells.size());

    RCLCPP_INFO(get_logger(), "Dividing cells according to the traversability");
    double variance = 0.0;
    double mean = 0.0;
    int ix = 0;
    double mahalanobis_thres_ = 0.4;
    while (variance < 1.0e-4)
    {
        double prev_mean = mean;
        mean = (ix*mean + cells[ix].value) / (ix + 1);
        variance = (ix*(variance + std::pow(prev_mean, 2.0)) + std::pow(cells[ix].value, 2.0)) / (ix + 1) - std::pow(mean, 2.0);
        ++ix;
    }
    RCLCPP_INFO(get_logger(), "Mean: %f, Variance: %f", mean, variance);

    for (; ix<cells.size(); ++ix)
    {
        double mahalanobis_dist = std::fabs(cells[ix].value - mean)/std::sqrt(variance);
        RCLCPP_INFO(get_logger(), "mahalanobis dist: %f", mahalanobis_dist);
        if (mahalanobis_dist > mahalanobis_thres_) 
        {
            break;
        }
        else 
        {
            // add cells[i] to current group
            double prev_mean = mean;
            mean = (mean*ix + cells[ix].value)/(ix+1);
            variance = (ix*(variance + std::pow(mean, 2.0)) + std::pow(cells[ix].value, 2.0)) / (ix+1) - std::pow(prev_mean, 2.0);
        }
    }
    RCLCPP_INFO(get_logger(), "Number of cells in the group: %d", ix);

    // remove value other than the group
    RCLCPP_INFO(get_logger(), "Remove value other than the group");
    _dst.setGeometry(_src.getLength(), _src.getResolution(), _src.getPosition());
    _dst.setFrameId(_src.getFrameId());
    _dst.setTimestamp(_src.getTimestamp());
    _dst.add("traversability", _src.get("traversability"));
    _dst.add("traversability1", _src.get("traversability"));
    for (int j=ix; j<cells.size(); ++j)
    {
        _dst.at("traversability1", cells[j].index) = NAN;
    }

    return true;
}

bool PlaneExtractor::dividePlane(grid_map::GridMap& _src, const std::string& input_layer, const std::string& output_layer)
{
    // get amp
    auto& map = _src.get(input_layer);
    
    // convert to cv gray image (unsigned int8_t, 1 channel)
    cv::Mat gray, binary;
    grid_map::GridMapCvConverter::toImage<uint8_t, 1>(_src, input_layer, CV_8UC1, gray);
    cv::threshold(gray, binary, 0, 255, cv::THRESH_BINARY); // binary

    // Find contours
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(binary, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

    // Fill inside contours
    cv::Scalar color(255); // white color
    for (int idx=0; idx>=0; idx = hierarchy[idx][0])
    {
        cv::Mat img(binary.size(), CV_8UC1, cv::Scalar(0)); // black color
        cv::drawContours(img, contours, idx, color, CV_FILLED, 8, hierarchy);
        // convert to grid map
        grid_map::GridMapCvConverter::addLayerFromImage<uint8_t, 1>
            (img, output_layer+std::to_string(idx), _src);
    }

    return true;
}


bool PlaneExtractor::divideByNormalVector(const grid_map::GridMap& _src, grid_map::GridMap& _dst)
{
    
}

void PlaneExtractor::publishGridMap(const grid_map::GridMap& _src)
{
    grid_map_msgs::msg::GridMap::UniquePtr msg(new grid_map_msgs::msg::GridMap);
    msg = grid_map::GridMapRosConverter::toMessage(_src);
    pub_grid_map_->publish(std::move(msg));
}

} // namespace plane_extraction


#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(plane_extraction::PlaneExtractor)