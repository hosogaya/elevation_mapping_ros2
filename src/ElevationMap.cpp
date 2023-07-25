#include <elevation_mapping_ros2/ElevationMap.hpp>

namespace elevation_mapping
{

ElevationMap::ElevationMap(const rclcpp::Node* _node)
    : raw_map_({"elevation", "variance", "horizontal_variance_x", "horizontal_variance_y", "horizontal_varizance_xy", 
        "color", "time", "dynamic_time", "lowest_scan_point", "sensor_x_at_lowest_scan", "sensory_at_lowest_scan", "sensor_z_at_lowest_scan"}), 
      fused_map_({"elevation", "upper_bound", "lower_bound", "color"})
{
    raw_map_.setBasicLayers({"elevation", "variance"});
    fused_map_.setBasicLayers({"elevation", "upper_bound", "lower_bound", "color"});
    clear();

    system_clock_ = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
    ros_clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    logger_ = _node->get_logger();
}

bool ElevationMap::add(PointCloudType::Ptr _point_cloud, Eigen::VectorXf& _variance, const rclcpp::Time& _time_stamp, const Eigen::Affine3d& _transform_sensor2map_)
{
    const rclcpp::Time method_start_time = system_clock_->now(); // @todo convert to wall clock

    // update initial time if it is not initiallized.
    if (initial_time_.seconds() == 0)
    {
        initial_time_ = _time_stamp;
    }
    const float scan_time_since_initialization = static_cast<float>((_time_stamp - initial_time_).seconds());
    const float current_time = static_cast<float>(ros_clock_->now().seconds());

    auto& elevation_layer = raw_map_["elevation"];
    auto& variance_layer = raw_map_["variance"];
    auto& horizontal_variance_x_layer = raw_map_["horizontal_variance_x"];
    auto& horizontal_variance_y_layer = raw_map_["horizontal_variance_y"];
    auto& horizontal_variance_xy_layer = raw_map_["horizontal_variance_xy"];
    auto& color_layer = raw_map_["color"];
    auto& time_layer = raw_map_["time"];
    auto& dynamic_time_layer = raw_map_["dynamic_time"];
    auto& lowest_scan_point_layer = raw_map_["lowest_scan_point"];
    auto& sensor_x_at_lowest_scan_layer = raw_map_["sensor_x_at_lowest_scan"];
    auto& sensor_y_at_lowest_scan_layer = raw_map_["sensor_y_at_lowest_scan"];
    auto& sensor_z_at_lowest_scan_layer = raw_map_["sensor_z_at_lowest_scan"];

    std::vector<Eigen::Ref<const grid_map::Matrix>> basic_layer;
    for (const std::string& layer: raw_map_.getBasicLayers())
    {
        basic_layer.emplace_back(raw_map_.get(layer));
    }

    // for all points
    for (size_t i=0; i<_point_cloud->size(); ++i)
    {
        auto& point = _point_cloud->points[i];
        grid_map::Index index;
        grid_map::Position position(point.x, point.y);
        // Skip if it does not lie within the elevation map
        if (!raw_map_.getIndex(position, index))
        {
            continue;
        }

        auto& elevation = elevation_layer(index(0), index(1));
        auto& variance = variance_layer(index(0), index(1));
        auto& horizontal_variance_x = horizontal_variance_x_layer(index(0), index(1));
        auto& horizontal_variance_y = horizontal_variance_y_layer(index(0), index(1));
        auto& horizontal_variance_xy = horizontal_variance_xy_layer(index(0), index(1));
        auto& color = color_layer(index(0), index(1));
        auto& time = time_layer(index(0), index(1));
        auto& dynaic_time = dynamic_time_layer(index(0), index(1));
        auto& lowest_scan_point = lowest_scan_point_layer(index(0), index(1));
        auto& sensor_x_at_lowest_scan = sensor_x_at_lowest_scan_layer(index(0), index(1));
        auto& sensor_y_at_lowest_scan = sensor_y_at_lowest_scan_layer(index(0), index(1));
        auto& sensor_z_at_lowest_scan = sensor_z_at_lowest_scan_layer(index(0), index(1));
    
        const float point_variance = _variance(i);
        bool is_vaild = std::all_of(basic_layer.begin(), basic_layer.end(), [&](Eigen::Ref<const grid_map::Matrix> layer){return std::isfinite(layer(index(0), index(1)));});

        if (!is_vaild)
        {
            elevation = point.z;
            variance = point_variance;
            horizontal_variance_x = min_horizontal_variance_;
            horizontal_variance_y = min_horizontal_variance_;
            horizontal_variance_xy = 0.0;
            grid_map::colorValueToVector(point.getRGBVector3i(), color);
            continue;
        }

        const double mahalanobis_distance = std::fabs(point.z - elevation)/std::sqrt(variance);
        if (mahalanobis_distance > mahalanobis_distance_thres_)
        {
            if (scan_time_since_initialization - time <= scanning_duration_ && elevation > point.z)
            {
                // Ignore point if measurement is from the same point cloud (time comparison) and 
                // if measurement is lower than the elevation in the map
            }
            else if (scan_time_since_initialization - time <= scanning_duration_)
            {
                elevation = increase_height_alpha_*elevation + (1.0-increase_height_alpha_)*point.z;
                variance = increase_height_alpha_*variance + (1.0-increase_height_alpha_)+point_variance;
            }
            else 
            {
                variance += multi_height_noise_; // fast adaptation to changing terrain
            }
            continue;
        }   

        // store lowest points from scan for visibility checking 
        const float point_height_plus_uncertainty = point.z + 3.0*std::sqrt(point_variance); // 3*sigma (99% confidence interval)
        if (std::isnan(lowest_scan_point) || point_height_plus_uncertainty < lowest_scan_point)
        {
            lowest_scan_point = point_height_plus_uncertainty;
            const grid_map::Position3 sensor_translation(_transform_sensor2map_.translation());
            sensor_x_at_lowest_scan = sensor_translation.x();
            sensor_y_at_lowest_scan = sensor_translation.y();
            sensor_z_at_lowest_scan = sensor_translation.z();
        }

        elevation = (variance*point.z + point_variance*elevation) / (point_variance + variance);
        variance = (point_variance*variance) / (point_variance + variance);
        // @todo add color fusion
        grid_map::colorVectorToValue(point.getRGBVector3i(), color);
        time = scan_time_since_initialization;
        dynaic_time = current_time;

        // horizontal varicance are reset
        horizontal_variance_x = min_horizontal_variance_;
        horizontal_variance_y = min_horizontal_variance_;
        horizontal_variance_xy = 0.0;
    } // loop for point cloud

    clean();
    raw_map_.setTimestamp(_time_stamp.nanoseconds());
    const rclcpp::Duration duration = system_clock_->now() - method_start_time;
    RCLCPP_DEBUG(logger_, "Raw map has been updated with a new point in %f s", duration.seconds());

    return true;
}

bool ElevationMap::fuseAll()
{
    return fuse(grid_map::Index(0, 0), fused_map_.getSize());
}

bool ElevationMap::fuse(const grid_map::Index& _top_left_index, const grid_map::Size& _size)
{
    if ((_size == 0).any()) return false;

    // initialization
    const rclcpp::Time method_start_time = system_clock_->now();

    auto copy_raw_map = raw_map_;
    const double half_resolution = fused_map_.getResolution() / 2.0;
    const float minimal_weight = std::numeric_limits<float>::epsilon()*static_cast<float>(2.0);
    // coservative cell inclusion for ellipse iterator
    const double ellipse_extension = M_SQRT2 * fused_map_.getResolution();

    // Check if there is the need to reset out-dated data.
    if (fused_map_.getTimestamp() != copy_raw_map.getTimestamp())
    {
        resetFusedMap();
    }
    // Align fused map with raw map
    if (fused_map_.getPosition() != copy_raw_map.getPosition())
    {
        fused_map_.move(copy_raw_map.getPosition());
    }

    // for each cell
    for (grid_map::SubmapIterator area_iterator(copy_raw_map, _top_left_index, _size); !area_iterator.isPastEnd(); ++area_iterator)
    {
        if (fused_map_.isValid(*area_iterator)) continue;
        if (!copy_raw_map.isValid(*area_iterator)) continue;

        // get size of error epsilon 
        const float& sigma_x_square = copy_raw_map.at("horizontal_variance_x", *area_iterator);
        const float& sigma_y_square = copy_raw_map.at("horizontal_variance_y", *area_iterator);
        const float& sigma_xy_square = copy_raw_map.at("horizontal_variance_xy", *area_iterator);

        // Compute eigen values of ellipse
        Eigen::Matrix2d covariance_matrix;
        covariance_matrix << sigma_x_square, sigma_xy_square, sigma_xy_square, sigma_y_square;
        // 95.45% confidence ellipse is 2.486-sigma for 2d problem
        const double uncertainty_factor = 2.486; // sqrt(6.18)
        Eigen::EigenSolver<Eigen::Matrix2d> solver(covariance_matrix);
        Eigen::Array2d eigen_values(solver.eigenvalues().real().cwiseAbs());

        // Compute index of max/min eigen values of ellipse
        Eigen::Array2d::Index max_eigen_value_index(0);
        Eigen::Array2d::Index min_eigen_value_index(0);
        eigen_values.maxCoeff(&max_eigen_value_index);
        max_eigen_value_index == Eigen::Array2d::Index(0) ? min_eigen_value_index = 1 min_eigen_value_index = 0;
        const grid_map::Length ellipse_length = 2.0*uncertainty_factor * grid_map::Length(eigen_values(max_eigen_value_index), eigen_values(min_eigen_value_index)).sqrt() + ellipse_extension;
        // relative to longer axis
        const double ellipse_rotation = std::atan2(solver.eigenvalues().col(max_eigen_value_index).real()(1), solver.eigenvalues().col(max_eigen_value_index).real(0));

        // request length and posistion of submap in map
        grid_map::Position requested_submap_position;
        copy_raw_map.getPosition(*area_iterator, requested_submap_position);
        grid_map::EllipseIterator ellipse_iterator(copy_raw_map, requested_submap_position, ellipse_length, ellipse_rotation);

        // prepare data fusion
        Eigen::ArrayXf means, weights;
        const size_t max_number_of_cells_to_fuse = ellipse_iterator.getSubmapSize().prod();
        means.resize(max_number_of_cells_to_fuse);
        weights.resize(max_number_of_cells_to_fuse);
        WeightedEmpiricalCumulativeDistributionFunction<float> lower_bound_distribution;
        WeightedEmpiricalCumulativeDistributionFunction<float> upper_bound_distribution;

        float max_standard_deviation = std::sqrt(eigen_values(max_eigen_value_index));
        float min_standard_deviation = std::sqrt(eigen_values(min_eigen_value_index));
        Eigen::Rotation2D<double> rotation_matrix(ellipse_rotation);
        std::string min_eigen_value_layer;
        std::string max_eigen_value_layer;

        if (max_eigen_value_index == 0)
        {
            max_eigen_value_layer = "horizontal_variance_x";
            min_eigen_value_layer = "horizontal_variance_y";
        }
        else 
        {
            max_eigen_value_layer = "horizontal_variance_y";
            min_eigen_value_layer = "horizontal_variance_x";

        }

        size_t i = 0;
        for (; !ellipse_iterator.isPastEnd(); ++ellipse_iterator)
        {
            if (!copy_raw_map.isValid(*ellipse_iterator)) continue;

            means[i] = copy_raw_map.at("elevation", *ellipse_iterator);

            // compute weight probability
            grid_map::Position abusolute_position;
            copy_raw_map.getPosition(*ellipse_iterator, abusolute_position);
            Eigen::Vector2d distance_to_center = (rotation_matrix*(abusolute_position - requested_submap_position)).cwiseAbs();

            float probability1 = cumulativeDistributionFunction(distance_to_center.x() + half_resolution, 0.0, max_standard_deviation) 
                                -cumulativeDistributionFunction(distance_to_center.x() - half_resolution, 0.0, max_standard_deviation);
            float probability2 = cumulativeDistributionFunction(distance_to_center.y() + half_resolution, 0.0, min_standard_deviation) 
                                -cumulativeDistributionFunction(distance_to_center.y() - half_resolution, 0.0, min_standard_deviation);
        
            const float wieght = std::max(minimal_weight, probability1*probability2);
            weights[i] = wieght;
            const float standard_deviation = std::sqrt(copy_raw_map.at("variance", *ellipse_iterator));
            lower_bound_distribution.add(means[i] - 2.0*standard_deviation, weights);
            upper_bound_distribution.add(means[i] + 2.0*standard_deviation, weights);

            ++i;
        }
        if (i==0) 
        {
            fused_map_.at("elevation", *area_iterator) = copy_raw_map.at("elevation", *area_iterator);
            fused_map_.at("lower_bound", *area_iterator) = copy_raw_map.at("elevation", *area_iterator) - 2.0*std::sqrt(copy_raw_map.at("variance", *area_iterator));
            fused_map_.at("upper_bound", *area_iterator) = copy_raw_map.at("elevation", *area_iterator) + 2.0*std::sqrt(copy_raw_map.at("variance", *area_iterator));
            fused_map_.at("color", *area_iterator) = copy_raw_map.at("color", *area_iterator);
            continue;
        }
        
        means.conservativeResize(i);
        weights.coeffByOuterInner(i);

        float mean = (weights*means).sum()/ weights.sum();

        if (!std::isfinite(mean)) continue;
        
        // add to fused map
        fused_map_.at("elevation", *area_iterator) = mean;
        lower_bound_distribution.compute();
        upper_bound_distribution.compute();
        fused_map_.at("lower_bound", *area_iterator) = lower_bound_distribution.quantile(0.01);
        fused_map_.at("upper_bound", *area_iterator) = upper_bound_distribution.quantile(0.99);
        fused_map_.at("color", *area_iterator) = copy_raw_map.at("color", *area_iterator);
    } // end for each cell

    fused_map_.setTimestamp(copy_raw_map.getTimestamp());
    const rclcpp::Duration duration(system_clock_->now()-method_start_time);
    RCLCPP_DEBUG(logger_, "Elevation map has been fused in %f s", duration.seconds());

    return true;
} // end of fuse

void ElevationMap::visibilityCleanup(const rclcpp::Time& _time_stamp)
{
    const rclcpp::Time method_start_time = system_clock_->now();
    const doudble time_since_initialization = (last_update_time_ - initial_time_).seconds();

    // copy raw elevation map data
    visibility_clean_up_map_ = raw_map_;
    raw_map_.clear("lowest_scan_point");
    raw_map_.clear("sensor_x_at_lowest_scan");
    raw_map_.clear("sensor_y_at_lowest_scan");
    raw_map_.clear("sensor_xy_at_lowest_scan");

    visibility_clean_up_map_.add("max_height");
    
    // Create max ehight layer with ray tracing
    for (grid_map::GridMapIterator iterator(visibility_clean_up_map_); !iterator.isPastEnd(); ++iterator)
    {
        if (!visibility_clean_up_map_.isValid(*iterator)) continue;
        
        const auto& lowest_scan_point = visibility_clean_up_map_.at("lowest_scan_point", *iterator);
        const auto& sensor_x_at_lowest_point = visibility_clean_up_map_.at("sensor_x_at_lowest_scan", *iterator);
        const auto& sensor_y_at_lowest_point = visibility_clean_up_map_.at("sensor_y_at_lowest_scan", *iterator);
        const auto& sensor_z_at_lowest_point = visibility_clean_up_map_.at("sensor_z_at_lowest_scan", *iterator);

        if (std::isnan(lowest_scan_point)) continue;
        
        grid_map::Index index_at_sensor;
        if (!visibility_clean_up_map_.getIndex(grid_map::Position(sensor_x_at_lowest_point, sensor_y_at_lowest_point), index_at_sensor)) continue;

        grid_map::Position point;
        visibility_clean_up_map_.getPosition(*iterator, point);
        float point_diff_x = point.x() - sensor_x_at_lowest_point;
        float point_diff_y = point.y() - sensor_y_at_lowest_point;
        float distance_to_point = std::sqrt(std::pow(point_diff_x, 2.0) + std::pow(point_diff_y, 2.0));
        if (distance_to_point > 0.0)
        {
            for (grid_map::LineIterator line_iterator(visibility_clean_up_map_, index_at_sensor, *iterator); !line_iterator.isPastEnd(); ++line_iterator)
            {
                grid_map::Position cell_position;
                visibility_clean_up_map_.getPosition(*line_iterator, cell_position);
                const float cell_diff_x = cell_position.x() - sensor_x_at_lowest_point;
                const float cell_diff_y = cell_position.y() - sensor_y_at_lowest_point;
                const float distance_to_cell = distance_to_point - std::sqrt(std::pow(cell_diff_x, 2.0) + std::pow(cell_diff_y, 2.0));
                const float max_height_point = lowest_scan_point + (sensor_z_at_lowest_point - lowest_scan_point)/distance_to_point*distance_to_cell;
                auto& cell_max_height = visibilityCleanup.at("max_height", *line_iterator);
                if (std::isnan(cell_max_height) || cell_max_height > max_height_point)
                {
                    cell_max_height = max_height_point;
                }
            }// end of line iterator
        }// end of ray tracing for one point
    } // end of gird_map iterator (ray tracing)

    // vector of indeces that will be removed.
    std::vector<grid_map::Position> cell_position_to_remove;
    for (grid_map::GridMapIterator iterator(visibility_clean_up_map_); !iterator.isPastEnd(); ++iterator)
    {
        if (!visibility_clean_up_map_.isValid(*iterator)) continue;

        const auto& time = visibility_clean_up_map_.at("time", *iterator);
        if (time_since_initialization - time > scanning_duration_)
        {
            const auto& elevation = visibility_clean_up_map_.at("elevation", *iterator);
            const auto& variance = visibility_clean_up_map_.at("variance", *iterator);
            const auto& max_height = visibility_clean_up_map_.at("max_height", *iterator);

            if (!std::isnan(max_height) && elevation - 3.0*std::sqrt(variance) > max_height)
            {
                grid_map::Position position;
                visibility_clean_up_map_.getPosition(*iterator, position);
                cell_position_to_remove.push_back(position);
            }
        }
    } // end of grid map iterator (extract remove position)

    for (const auto& cell_position: cell_position_to_remove)
    {
        grid_map::Index index;
        if (!raw_map_.getIndex(cell_position, index)) continue;

        if (raw_map_.isValid(index))
        {
            raw_map_.at("elevation", index) = NAN;
            raw_map_.at("dynamic_time", index) = 0.0;
        }
    } // end of remove position

    rclcpp::Duration duration = system_clock_->now() - method_start_time;
    RCLCPP_DEBUG(logger_, "Visibility clean up has been performed in %f s (%d point)", duration.seconds(), (int)cell_position_to_remove.size());
}

bool ElevationMap::update(const grid_map::Matrix& _variance, const grid_map::Matrix& _horizontal_variance_x, const grid_map::Matrix& _horizontal_variance_y, const grid_map::Matrix& _horizontal_variance_xy, const rclcpp::Time& _time_stamp)
{
    const auto& size = raw_map_.getSize();

    if (!((grid_map::Index(_variance.rows(), _variance.cols()) == size).all() &&
        (grid_map::Index(_horizontal_variance_x.rows(), _horizontal_variance_x.cols()) == size).all() &&
        (grid_map::Index(_horizontal_variance_x.rows(), _horizontal_variance_y.cols()) == size).all() &&
        (grid_map::Index(_horizontal_variance_x.rows(), _horizontal_variance_xy.cols()) == size).all()))
    {
        RCLCPP_ERROR(logger_, "The size of the update matrices does not match.");
        return false;
    }   

    raw_map_.get("variance") += _variance;
    raw_map_.get("horizontal_variance_x") += _horizontal_variance_x;
    raw_map_.get("horizontal_variance_y") += _horizontal_variance_y;
    raw_map_.get("horizontal_variance_xy") += _horizontal_variance_xy;
    clean();
    raw_map_.setTimestamp(_time_stamp.nanoseconds());
    
    return true;
}

bool ElevationMap::clear()
{
    raw_map_.clearAll();
    raw_map_.resetTimestamp();
    raw_map_.get("dynamic_time").setZero();

    resetFusedMap();
    return true;
}

void ElevationMap::resetFuesdMap() 
{
    fused_map.clearAll();
    fused_map.resetTimestamp();
}

bool ElevationMap::clean()
{
    raw_map_.get("variance") = raw_map_.get("variance").unaryExpr(VarianceClampOperator(min_normal_variance_, max_normal_variance_));
    raw_map_.get("horizontal_variance_x") = raw_map_.get("horizontal_variance_x").unaryExpr(VarianceClampOperator(min_horizontal_variance_, max_horizontal_variance_));
    raw_map_.get("horizontal_variance_y") = raw_map_.get("horizontal_variance_y").unaryExpr(VarianceClampOperator(min_horizontal_variance_, max_horizontal_variance_));

    return true;
}

void ElevationMap::move(const grid_map::Position& position)
{
    std::vector<grid_map::BufferRegion> new_region;
    if (raw_map_.move(position, new_region))
    {
        RCLCPP_DEBUG(logger_, "Elevatino map has been moved to position (%f, %f)", position.x(), position.y());

        // "dynamic time" layer is meant to be interpreted as integer values, therefore nan:s need to be zero.
        grid_map::Matrix& dynamic_time{raw_map_.get("dynamic_time")};
        dynamic_time = dynamic_time.array().isNaN().select(grid_map::Matrix::Scalar(0.0f), dynamic_time.array());
    }
}

const std::string& ElevationMap::getFrameID() const
{
    return raw_map_.getFrameId();
}

const rclcpp::Time& ElevationMap::getTimeOfLastUpdate() const 
{
    return last_update_time_;
}

GridMap& ElevationMap::getRawMap()  
{
    return raw_map_;
}

GridMap& ElevationMap::getFusedMap()  
{
    return fused_map_;
}

float ElevationMap::cumulativeDistributionFunction(float x, float mean, float standardDeviation) {
  return 0.5 * std::erfc(-(x - mean) / (standardDeviation * std::sqrt(2.0)));
}

void ElevationMap::readParameter(rclcpp::Node* _node)
{
    _node->declare_parameter("min_normal_variance", min_normal_variance_, 5.0);
    _node->declare_parameter("max_normal_variance", max_normal_variance_, 100.0);
    _node->declare_parameter("min_horizontal_variance", min_horizontal_variance_, 5.0);
    _node->declare_parameter("max_horizontal_variance", max_horizontal_variance_, 100.0);
    _node->declare_parameter("mahalanobis_distance_thres", mahalanobis_distance_thres_, 10.0);
    float scanning_duration_milliseconds;
    _node->declare_parameter("scanning_duration", scanning_duration_milliseconds, 0.1);
    scanning_duration_(scanning_duration_milliseconds*1e6);
    _node->declare_parameter("increase_height_alpha", increase_height_alpha_, 0.5);
    _node->declare_parameter("multi_height_noise", multi_height_noise_, 25.0);
    
}

}