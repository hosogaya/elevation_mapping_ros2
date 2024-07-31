#include <elevation_mapping_ros2/ElevationMap.hpp>

namespace elevation_mapping
{

ElevationMap::ElevationMap(Config config, std::string map_frame)
    : raw_map_({"elevation", "variance", "horizontal_variance_x", "horizontal_variance_y", "horizontal_variance_xy", 
        "time", "dynamic_time", "lowest_scan_point", "sensor_x_at_lowest_scan", "sensor_y_at_lowest_scan", "sensor_z_at_lowest_scan"}),
      fused_map_({"elevation", "upper_bound", "lower_bound"}), 
      logger_name_(config.logger_name_)
{
    raw_map_.setBasicLayers({"elevation", "variance"});
    fused_map_.setBasicLayers({"elevation", "upper_bound", "lower_bound"});
    clear();

    setGeometry(grid_map::Length(config.lateral_length_, config.longitudinal_length_), config.resolution_,
                grid_map::Position(config.initial_position_x_, config.initial_position_y_));
    setFrameID(map_frame);

    min_normal_variance_ = config.min_normal_variance_;
    max_normal_variance_ = config.max_nomral_variance_;
    min_horizontal_variance_ = config.min_horizontal_variance_;
    max_horizontal_variance_ = config.max_horizontal_variance_;
    mahalanobis_distance_thres_ = config.mahalanobis_distance_thres_;
    scanning_duration_ = rclcpp::Time(static_cast<size_t>(config.scanning_duration_*1e9)); // second->nanosecond
    increase_height_alpha_ = config.increase_height_alpha_;
    multi_height_noise_ = config.multi_height_noise_;
}

ElevationMap::~ElevationMap() {}

bool ElevationMap::add(PointCloudType::Ptr _point_cloud, Eigen::VectorXf& _variance, const rclcpp::Time& _time_stamp, const Eigen::Affine3d& _transform_sensor2map_)
{
    // update initial time if it is not initiallized.
    if (initial_time_.seconds() == 0)
    {
        initial_time_ = _time_stamp;
    }
    const rclcpp::Duration scan_time_since_initialization = _time_stamp - initial_time_;

    auto& elevation_layer = raw_map_["elevation"];
    auto& variance_layer = raw_map_["variance"];
    auto& horizontal_variance_x_layer = raw_map_["horizontal_variance_x"];
    auto& horizontal_variance_y_layer = raw_map_["horizontal_variance_y"];
    auto& horizontal_variance_xy_layer = raw_map_["horizontal_variance_xy"];
    // auto& color_layer = raw_map_["color"];
    auto& time_layer = raw_map_["time"];
    auto& dynamic_time_layer = raw_map_["dynamic_time"];
    auto& lowest_scan_point_layer = raw_map_["lowest_scan_point"];
    auto& sensor_x_at_lowest_scan_layer = raw_map_["sensor_x_at_lowest_scan"];
    auto& sensor_y_at_lowest_scan_layer = raw_map_["sensor_y_at_lowest_scan"];
    auto& sensor_z_at_lowest_scan_layer = raw_map_["sensor_z_at_lowest_scan"];

    // for all points
    size_t point_within_map_num = 0;
    for (size_t i=0; i<_point_cloud->size(); ++i)
    {
        auto& point = _point_cloud->points[i];
        if (std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z)) continue;
        grid_map::Index index;
        grid_map::Position position(point.x, point.y);
        // Skip if it does not lie within the elevation map
        if (!raw_map_.getIndex(position, index))
        {
            continue;
        }
        point_within_map_num++;

        auto& elevation = elevation_layer(index(0), index(1));
        auto& variance = variance_layer(index(0), index(1));
        auto& horizontal_variance_x = horizontal_variance_x_layer(index(0), index(1));
        auto& horizontal_variance_y = horizontal_variance_y_layer(index(0), index(1));
        auto& horizontal_variance_xy = horizontal_variance_xy_layer(index(0), index(1));
        auto& time = time_layer(index(0), index(1));
        auto& dynaic_time = dynamic_time_layer(index(0), index(1));
        auto& lowest_scan_point = lowest_scan_point_layer(index(0), index(1));
        auto& sensor_x_at_lowest_scan = sensor_x_at_lowest_scan_layer(index(0), index(1));
        auto& sensor_y_at_lowest_scan = sensor_y_at_lowest_scan_layer(index(0), index(1));
        auto& sensor_z_at_lowest_scan = sensor_z_at_lowest_scan_layer(index(0), index(1));
    
        const float point_variance = _variance(i);
        if (std::isnan(point_variance)) continue;
        if (!std::isfinite(elevation) || !std::isfinite(variance))
        {
            elevation = point.z;
            variance = point_variance;
            horizontal_variance_x = min_horizontal_variance_;
            horizontal_variance_y = min_horizontal_variance_;
            horizontal_variance_xy = 0.0;
            continue;
        }

        
        if (variance > 0.0) {
            const double mahalanobis_distance = std::fabs(point.z - elevation)/std::sqrt(variance);
            if (mahalanobis_distance > mahalanobis_distance_thres_)
            {
                if (scan_time_since_initialization.seconds() - time <= scanning_duration_.seconds() && elevation > point.z)
                {
                    // Ignore point if measurement is from the same point cloud (time comparison) and 
                    // if measurement is lower than the elevation in the map
                }
                else if (scan_time_since_initialization.seconds() - time <= scanning_duration_.seconds())
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
        }

        // store lowest points from scan for visibility checking 
        // sensor position at the time when scans the lowest (min z) point 
        const float point_height_plus_uncertainty = point.z + 3.0*std::sqrt(point_variance); // 3*sigma (99% confidence interval)
        if (std::isnan(lowest_scan_point) || point_height_plus_uncertainty < lowest_scan_point)
        {
            lowest_scan_point = point_height_plus_uncertainty;
            const grid_map::Position3 sensor_translation(_transform_sensor2map_.translation());
            sensor_x_at_lowest_scan = sensor_translation.x();
            sensor_y_at_lowest_scan = sensor_translation.y();
            sensor_z_at_lowest_scan = sensor_translation.z();
        }
        if (point_variance == 0.0) {
            elevation = point.z;
        }
        else if (point_variance > 0.0) 
        {
            elevation = (variance*point.z + point_variance*elevation) / (point_variance + variance);
            variance = (point_variance*variance) / (point_variance + variance);
        }

        // grid_map::colorVectorToValue(point.getRGBVector3i(), color);
        time = scan_time_since_initialization.seconds();
        dynaic_time = _time_stamp.seconds();

        // horizontal varicance are reset
        horizontal_variance_x = min_horizontal_variance_;
        horizontal_variance_y = min_horizontal_variance_;
        horizontal_variance_xy = 0.0;
    } // loop for point cloud

    RCLCPP_DEBUG(rclcpp::get_logger(logger_name_), "The number of points within the elevation map is %ld", point_within_map_num);

    clean();
    raw_map_.setTimestamp(_time_stamp.nanoseconds());

    return true;
}

bool ElevationMap::fuseAll()
{
    return fuse(grid_map::Index(0.0), fused_map_.getSize());
}

    
bool ElevationMap::fuse(const grid_map::Index& top_left_index, const grid_map::Index& size)
{
    if ((size == 0).any()) return false;

    const auto start_time = std::chrono::system_clock::now();

    // Preparation
    const double half_resolution = fused_map_.getResolution()*0.5;
    const float minimal_weight = std::numeric_limits<float>::epsilon()*2.0f;
    const double ellipse_extension = M_SQRT2*fused_map_.getResolution();

    // Check if there is the need to reset out-dated data. 
    if (fused_map_.getTimestamp() != raw_map_.getTimestamp())
    {
        fused_map_.clearAll();
        fused_map_.resetTimestamp();
    }

    // Align fused map with raw map
    if (raw_map_.getPosition() != fused_map_.getPosition())
    {
        if (!fused_map_.move(raw_map_.getPosition()))
        {
            RCLCPP_WARN(rclcpp::get_logger(logger_name_), "Failed to move fused map to the same position as raw map");
            return false;
        }
        if (!fused_map_.isDefaultStartIndex()) fused_map_.convertToDefaultStartIndex();
    }


    // For each cell is requested area
    for (grid_map::SubmapIterator iterator(raw_map_, top_left_index, size); 
        !iterator.isPastEnd(); ++iterator
    )
    {
        // Check if fusion for this cell has already been done earlier
        if (fused_map_.isValid(*iterator)) continue;
        // Check source data is valid
        if (!raw_map_.isValid(*iterator)) continue;

        // Get size of error ellipse
        const float& sigma_x = raw_map_.at("horizontal_variance_x", *iterator);
        const float& sigma_y = raw_map_.at("horizontal_variance_y", *iterator);
        const float& sigma_xy = raw_map_.at("horizontal_variance_xy", *iterator);

        Eigen::Matrix2d covariance_matrix{{sigma_x, sigma_xy}, {sigma_xy, sigma_y}};
        // 95.45% confidence ellipse which is 2.486-sigma for 2 dof problem.
        // http://www.reid.ai/2012/09/chi-squared-distribution-table-with.html
        const double uncertainty_factor{2.486};

        Eigen::EigenSolver<Eigen::Matrix2d> solver(covariance_matrix);
        Eigen::Array2d eigen_values(solver.eigenvalues().real().cwiseAbs());

        Eigen::Array2d::Index max_eigen_value_index{0};
        eigen_values.maxCoeff(&max_eigen_value_index);
        Eigen::Array2d::Index min_eigen_value_index{0};
        if (max_eigen_value_index == Eigen::Array2d::Index(0)) min_eigen_value_index = 1;
        else min_eigen_value_index = 0;

        const grid_map::Length ellipse_length = 2.0*uncertainty_factor
                                            *grid_map::Length(eigen_values(max_eigen_value_index), eigen_values(min_eigen_value_index)).sqrt() 
                                            + ellipse_extension;

        const double ellipse_rotation = std::atan2(solver.eigenvectors().col(max_eigen_value_index).real()(1),
                                                solver.eigenvectors().col(max_eigen_value_index).real()(0));

        // requested length and position (center) of submap in map. 
        grid_map::Position requested_submap_position;
        raw_map_.getPosition(*iterator, requested_submap_position);
        
        // Prepare data fusion
        grid_map::EllipseIterator ellipse_iterator(raw_map_, requested_submap_position, ellipse_length, ellipse_rotation);
        const unsigned int max_number_of_cells_to_fuse = ellipse_iterator.getSubmapSize().prod();
        Eigen::ArrayXf means(max_number_of_cells_to_fuse);
        Eigen::ArrayXf weights(max_number_of_cells_to_fuse);
        WeightedEmpiricalCumulativeDistributionFunction<float> lower_bound_distribution;
        WeightedEmpiricalCumulativeDistributionFunction<float> upper_bound_distribution;    

        float max_standard_deviation = sqrt(eigen_values(max_eigen_value_index));
        float min_standard_deviation = sqrt(eigen_values(min_eigen_value_index));
        Eigen::Rotation2Dd rotation_matrix(ellipse_rotation);

        // for each cell in error ellipse
        size_t i{0};
        for (; !ellipse_iterator.isPastEnd(); ++ellipse_iterator)
        {
            if (!raw_map_.isValid(*ellipse_iterator)) continue;

            means[i] = raw_map_.at("elevation", *ellipse_iterator);

            // Compute weight from probability. 
            grid_map::Position aubsolute_position;
            raw_map_.getPosition(*ellipse_iterator, aubsolute_position);
            Eigen::Vector2d distance_to_center = (rotation_matrix*(aubsolute_position - requested_submap_position)).cwiseAbs();

            float probability1 = cumulativeDistributionFunction(distance_to_center.x() + half_resolution, 0.0, max_standard_deviation)
                                -cumulativeDistributionFunction(distance_to_center.x() - half_resolution, 0.0, max_standard_deviation);
            float probability2 = cumulativeDistributionFunction(distance_to_center.y() + half_resolution, 0.0, min_standard_deviation)
                                -cumulativeDistributionFunction(distance_to_center.y() - half_resolution, 0.0, min_standard_deviation);

            const float weight = std::max(minimal_weight, probability1*probability2);
            weights[i] = weight;

            const float standard_deviation = std::sqrt(raw_map_.at("variance", *ellipse_iterator));
            lower_bound_distribution.add(means[i] - 2.0*standard_deviation, weight);
            upper_bound_distribution.add(means[i] + 2.0*standard_deviation, weight);

            ++i;
        }

        if (i == 0)
        {
            // Nothing to fuse
            fused_map_.at("elevation", *iterator) = raw_map_.at("elevation", *iterator);
            fused_map_.at("lower_bound", *iterator) = raw_map_.at("elevation", *iterator) - 2.0*std::sqrt(raw_map_.at("variance", *iterator));
            fused_map_.at("upper_bound", *iterator) = raw_map_.at("elevation", *iterator) + 2.0*std::sqrt(raw_map_.at("variance", *iterator));

            continue;
        }

        // Fuse
        means.conservativeResize(i);
        weights.conservativeResize(i);
        float mean = (weights*means).sum()/weights.sum();

        if (!std::isfinite(mean))
        {
            RCLCPP_ERROR(rclcpp::get_logger(logger_name_), "Something went wrong when fusing the map");
            continue;
        }

        // add to the fused map
        fused_map_.at("elevation", *iterator) = mean;
        lower_bound_distribution.compute();
        upper_bound_distribution.compute();
        fused_map_.at("lower_bound", *iterator) = lower_bound_distribution.quantile(0.01);
        fused_map_.at("upper_bound", *iterator) = upper_bound_distribution.quantile(0.99);
    }
    fused_map_.setTimestamp(raw_map_.getTimestamp());
    const auto end_time = std::chrono::system_clock::now();
    double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
    RCLCPP_INFO(rclcpp::get_logger(logger_name_), "Map fusing takes %lf [ms]", elapsed);


    return true;
}

void ElevationMap::visibilityCleanup(const rclcpp::Time& _time_stamp)
{
    auto start = std::chrono::system_clock::now();
    RCLCPP_INFO(rclcpp::get_logger(logger_name_), "Calling visibility clean up");
    const rclcpp::Duration time_since_initialization = _time_stamp - initial_time_;

    // copy raw elevation map data
    visibility_clean_up_map_ = raw_map_;
    raw_map_.clear("lowest_scan_point");
    raw_map_.clear("sensor_x_at_lowest_scan");
    raw_map_.clear("sensor_y_at_lowest_scan");
    raw_map_.clear("sensor_z_at_lowest_scan");

    visibility_clean_up_map_.add("max_height");
    
    // Create max height layer with ray tracing
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
                auto& cell_max_height = visibility_clean_up_map_.at("max_height", *line_iterator);
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
        // If the cell has been not scanned in "scannning_duration_", it will be cleared 
        if (time_since_initialization.seconds() - time > scanning_duration_.seconds())
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

    auto end = std::chrono::system_clock::now();
    double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    RCLCPP_INFO(rclcpp::get_logger(logger_name_), "Finished visibility clean up. It takes %lf [ms]", elapsed);
}

bool ElevationMap::update(const grid_map::Matrix& _variance, const grid_map::Matrix& _horizontal_variance_x, const grid_map::Matrix& _horizontal_variance_y, const grid_map::Matrix& _horizontal_variance_xy, const rclcpp::Time& _time_stamp)
{
    const auto& size = raw_map_.getSize();

    if (!((grid_map::Index(_variance.rows(), _variance.cols()) == size).all() &&
        (grid_map::Index(_horizontal_variance_x.rows(), _horizontal_variance_x.cols()) == size).all() &&
        (grid_map::Index(_horizontal_variance_x.rows(), _horizontal_variance_y.cols()) == size).all() &&
        (grid_map::Index(_horizontal_variance_x.rows(), _horizontal_variance_xy.cols()) == size).all()))
    {
        RCLCPP_ERROR(rclcpp::get_logger(logger_name_), "The size of the update matrices does not match.");
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

    return true;
}


bool ElevationMap::clean()
{
    raw_map_.get("variance") = raw_map_.get("variance").unaryExpr(VarianceClampOperator<float>(min_normal_variance_, max_normal_variance_));
    raw_map_.get("horizontal_variance_x") = raw_map_.get("horizontal_variance_x").unaryExpr(VarianceClampOperator<float>(min_horizontal_variance_, max_horizontal_variance_));
    raw_map_.get("horizontal_variance_y") = raw_map_.get("horizontal_variance_y").unaryExpr(VarianceClampOperator<float>(min_horizontal_variance_, max_horizontal_variance_));

    return true;
}

void ElevationMap::move(const grid_map::Position& position)
{
    if (raw_map_.move(position))
    {
        if (!raw_map_.isDefaultStartIndex()) raw_map_.convertToDefaultStartIndex();
        RCLCPP_DEBUG(rclcpp::get_logger(logger_name_), "Elevatino map has been moved to position (%f, %f)", position.x(), position.y());

        // "dynamic time" layer is meant to be interpreted as integer values, therefore nan:s need to be zero.
        grid_map::Matrix& dynamic_time{raw_map_.get("dynamic_time")};
        dynamic_time = dynamic_time.array().isNaN().select(grid_map::Matrix::Scalar(0.0f), dynamic_time.array());
    }
}


bool ElevationMap::extractVaildArea(const GridMap& _src_map, GridMap& _dst_map)
{
    bool is_success = true;
    grid_map::Index bottomRight{0, 0};
    grid_map::Index topLeft = _src_map.getSize();
    for (grid_map::GridMapIterator iterator(_src_map); !iterator.isPastEnd(); ++iterator)
    {
        if (_src_map.isValid(*iterator, _src_map.getBasicLayers()))
        {
            if ((*iterator).x() > bottomRight.x()) bottomRight.x() = (*iterator).x();
            if ((*iterator).y() > bottomRight.y()) bottomRight.y() = (*iterator).y();
            if ((*iterator).x() < topLeft.x()) topLeft.x() = (*iterator).x();
            if ((*iterator).y() < topLeft.y()) topLeft.y() = (*iterator).y();
        }
    }
    grid_map::Index center_index = (topLeft + bottomRight)/2;
    grid_map::Position center_pos;
    _src_map.getPosition(center_index, center_pos);
    grid_map::Length length = _src_map.getResolution()*(bottomRight - topLeft).cast<double>();
    _dst_map = _src_map.getSubmap(center_pos, length, is_success);
    return is_success;
}


void ElevationMap::setGeometry(const grid_map::Length& _length, const double& _resolution, const grid_map::Position& _position)
{
    raw_map_.setGeometry(_length, _resolution, _position);
    fused_map_.setGeometry(_length, _resolution, _position);
    visibility_clean_up_map_.setGeometry(_length, _resolution, _position);
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger(logger_name_), "Elevation map grid resized to " << raw_map_.getSize()(0) << " rows and" << raw_map_.getSize()(1) << "columns."
        << " \n Resolution is " << raw_map_.getResolution() << ".\nLateral length " << raw_map_.getLength()(0) << ". Longitudinal length" << raw_map_.getLength()(1) << ".");
}

void ElevationMap::setFrameID(const std::string& _frame_id)
{
    raw_map_.setFrameId(_frame_id);
    fused_map_.setFrameId(_frame_id);
    visibility_clean_up_map_.setFrameId(_frame_id);
}

const std::string& ElevationMap::getFrameID() const
{
    return raw_map_.getFrameId();
}

const rclcpp::Time ElevationMap::getTimeOfLastUpdate(rcl_clock_type_t type) const 
{
    return rclcpp::Time(raw_map_.getTimestamp(), type);
}

float ElevationMap::cumulativeDistributionFunction(float x, float mean, float standardDeviation) {
  return 0.5 * std::erfc(-(x - mean) / (standardDeviation * std::sqrt(2.0)));
}

}