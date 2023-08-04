#include <plane_extraction/PlaneExtractor.h>

namespace plane_extraction
{
using namespace std::placeholders;
PlaneExtractor::PlaneExtractor() : rclcpp::Node("plane_extractor")
{
    sub_grid_map_ = create_subscription<grid_map_msgs::msg::GridMap>(
        "input/grid_map", 10, std::bind(PlaneExtractor::callbackGridMap, this, _1)
    );
}

PlaneExtractor::~PlaneExtractor() {}

void PlaneExtractor::callbackGridMap(const grid_map_msgs::msg::GridMap::UniquePtr _msg)
{
    grid_map::GridMap src_map;
    grid_map::GridMapRosConverter::fromMessage(*_msg, src_map);

    grid_map::GridMap sub_map;
    extractWithGeometry(src_map, sub_map);

}

bool PlaneExtractor::extractWithGeometry(const grid_map::GridMap& _src, grid_map::GridMap& _dst)
{
    //@TODO How to deremine center and length
    grid_map::Position center{0.0, 0.0};
    grid_map::Length length{1.0, 1.0};
    bool is_success = false;
    _dst = _src.getSubmap(center, length, is_success);
}

bool PlaneExtractor::extractWithTraversability(const grid_map::GridMap& _src, std::vector<grid_map::GridMap>& _dst)
{
    std::vector<std::pair<grid_map::Index, double>> cells;
    for (grid_map::GridMapIterator iterator; !iterator.isPastEnd(); ++iterator)
    {
        double value = _src.at("traversability", *iterator);
        if (std::isnan(value)) continue;
        cells.emplace(*iterator, value);
    }
    std::sort(cells.begin(), cells.end(), [](const auto& x, const auto& y){return x.second > y.second;});
    
    double variance = 0.0;
    double mean = 0.0;
    int n = 0;
    while (variance == 0.0)
    {
        mean += cells[n].second;
        variance = 0.0;
        double temp_mean = mean/(n+1);
        for (int i=0; i<n; ++i) 
        {
            variance += std::pow(cells[n].second - temp_mean, 2.0);
        }
        ++n;
    }
    mean /= n;
    variance /= n;

    double mahalanobis_thres_;
    int i=n;
    for (; i<cells.size(); ++i)
    {
        double mahalanobis_dist = std::fabs(cells[i].sencond - mean)/std::sqrt(variance);
        if (mahalanobis_dist > mahalanobis_thres_) 
        {
            break;
        }
        else 
        {
            // add cells[i] to current group
            double prev_mean = mean;
            mean = (mean*i + cells[i].second)/(i+1);
            variance = (i*(variance + std::pow(mean, 2.0)) + std::pow(cells[i].second, 2.0)) / (i+1) - std::pow(prev_mean, 2.0);
        }
    }
    
    // remove value other than the group
    _dst.emplace_back(_src);
    for (int j=i; i<cells.size(); ++j)
    {
        _dst.at("traversability", cells[j].first) = NAN;
    }

}

} // namespace plane_extraction
