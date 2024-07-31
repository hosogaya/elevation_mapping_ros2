#include <elevation_mapping_ros2/ElevationMapping.hpp>

namespace elevation_mapping {

using std::placeholders::_1;

ElevationMapping::ElevationMapping(const rclcpp::NodeOptions options) 
    // : rclcpp::Node("elevation_mapping", rclcpp::NodeOptions().use_intra_process_comms(true))
    : rclcpp::Node("elevation_mapping", options)
{
    readParameters();

    RCLCPP_INFO(get_logger(), "Finished reading parameters");
    // create tf listener
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    // subscriber
    // sub_point_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    //     // "input/point_cloud", 10, std::bind(&ElevationMapping::callbackPointcloud, this, _1)
    //     "input/point_cloud", rclcpp::QoS(10), 
    //     [this](const sensor_msgs::msg::PointCloud2::UniquePtr msg)->void
    //     {
    //         RCLCPP_INFO(get_logger(), "Calling point cloud");
    //         this->callbackPointcloud(*msg, 1);
    //     }
    // );

    for (int i=0; i<sensor_processors_.size(); ++i)
    {
        sub_point_clouds_.push_back(
            this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "input/point_cloud"+std::to_string(i+1), rclcpp::QoS(10), 
                [this, i](const sensor_msgs::msg::PointCloud2::UniquePtr msg) ->void
                {
                    this->callbackPointcloud(*msg, i);
                }
            )
        );
    }

    pub_raw_map_ = this->create_publisher<grid_map_msgs::msg::GridMap>(
        "output/raw_map", 10
    );  

    if (use_pose_update_)
    {
        RCLCPP_INFO(get_logger(), "Updation by pose message is enabled");
        // std::string input_pose_topic = declare_parameter("input.pose_covariance");
        sub_pose_.subscribe(this, "/input/pose");
        pose_cache_.connectInput(sub_pose_);
        pose_cache_.setCacheSize(pose_cache_size_);
    }
    else 
    {
        RCLCPP_INFO(get_logger(), "Updation by pose message is disable");
    }

    if (use_visibility_clean_up_ && visibility_clean_up_duration_ > 0.0)
    {
        auto dt = std::chrono::microseconds(size_t(visibility_clean_up_duration_*1e6));
        visibility_clean_up_timer_ = create_wall_timer(dt, std::bind(&ElevationMapping::visibilityCleanUpCallback, this));
    }
    clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
}

ElevationMapping::~ElevationMapping() {}

void ElevationMapping::callbackPointcloud(const sensor_msgs::msg::PointCloud2& _point_cloud, const int topic_index)
{
    RCLCPP_DEBUG(get_logger(), "Calling callback point cloud No. %d", topic_index);
    std::chrono::system_clock::time_point begin = std::chrono::system_clock::now();
    last_point_cloud_update_time_ = rclcpp::Time(_point_cloud.header.stamp);

    Eigen::Matrix<double, 6, 6> robot_pose_covariance;
    robot_pose_covariance.setZero();
    if (use_pose_update_)
    {
        std::shared_ptr<geometry_msgs::msg::PoseWithCovarianceStamped const> pose_msg = pose_cache_.getElemBeforeTime(last_point_cloud_update_time_);
        if (!pose_msg) 
        {
            if (pose_cache_.getOldestTime().nanoseconds() > last_point_cloud_update_time_.nanoseconds())
            {
                RCLCPP_ERROR(this->get_logger(), "The oldest pose available is at %f, requested pose at %f", pose_cache_.getOldestTime().seconds(), last_point_cloud_update_time_.seconds());
            }
            else 
            {
                RCLCPP_ERROR(this->get_logger(), "Could not get pose information from robot for time %f", last_point_cloud_update_time_.seconds());
            }
            return;
        }
        robot_pose_covariance = Eigen::Map<const Eigen::MatrixXd>(pose_msg->pose.covariance.data(), 6, 6);
    }

    // process point cloud
    auto s_pc_process = std::chrono::system_clock::now();
    PointCloudType::Ptr point_cloud_map_frame(new PointCloudType);
    Eigen::VectorXf height_variance;
    if (!sensor_processors_[topic_index]->process(_point_cloud, robot_pose_covariance, point_cloud_map_frame, height_variance))
    {
        if (!sensor_processors_[topic_index]->isFirstTfAvailable())
        {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 10, "Waiting for tf transformation to be available. (Message is throttled. 10s)");
            return;
        }
        else 
        {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 10, "Point cloud could not be processed. (Throttled 10s)");
            return;
        }
    }
    auto e_pc_process = std::chrono::system_clock::now();
    double elapsed_pc_process = std::chrono::duration_cast<std::chrono::milliseconds>(e_pc_process - s_pc_process).count();
    RCLCPP_DEBUG(get_logger(), "Point Cloud processing time: %lf ms", elapsed_pc_process);

    updateMapLocation();

    if (!updatePrediction(last_point_cloud_update_time_))
    {
        RCLCPP_ERROR(get_logger(), "Updating process noise failed");
        return ;
    }

    // add point cloud to elevation map
    auto s_add = std::chrono::system_clock::now();
    if (!map_->add(point_cloud_map_frame, height_variance, last_point_cloud_update_time_, sensor_processors_[topic_index]->getTransformSensor2Map()))
    {
        RCLCPP_ERROR(get_logger(), "Adding point cloud to elevation map failed");
        // reset map update timer
        return ;
    }    
    auto e_add = std::chrono::system_clock::now();
    double elapsed_add = std::chrono::duration_cast<std::chrono::milliseconds>(e_add - s_add).count();
    RCLCPP_DEBUG(get_logger(), "Add point Cloud time: %lf ms", elapsed_add);

    if (use_and_publish_fused_map_)
    {
        map_->fuseAll(); 
        grid_map_msgs::msg::GridMap::UniquePtr message(new grid_map_msgs::msg::GridMap);
        message = grid_map::GridMapRosConverter::toMessage(map_->getFusedMap(), map_->getFusedMap().getBasicLayers());
        pub_raw_map_->publish(std::move(message));
    }
    else 
    {
        grid_map_msgs::msg::GridMap::UniquePtr message(new grid_map_msgs::msg::GridMap);
        message = grid_map::GridMapRosConverter::toMessage(map_->getRawMap(), std::vector<std::string>{"elevation", "variance"});
        pub_raw_map_->publish(std::move(message));
    }

    std::chrono::system_clock::time_point end = std::chrono::system_clock::now();
    double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
    RCLCPP_DEBUG(get_logger(), "elevation mapping processing time: %lf ms", elapsed);
}

bool ElevationMapping::updateMapLocation()
{
    RCLCPP_DEBUG(this->get_logger(), "Elevation map is checked for relocalization");

    geometry_msgs::msg::TransformStamped track_point;
    track_point.header.frame_id = track_point_frame_id_;
    track_point.header.stamp.sec = rclcpp::Time(0).seconds();
    track_point.header.stamp.nanosec = rclcpp::Time(0).nanoseconds();
    track_point.transform.translation.x = 0.0;
    track_point.transform.translation.y = 0.0;
    track_point.transform.translation.z = 0.0;

    geometry_msgs::msg::TransformStamped transformed_track_point;
    try
    {
        // get transformed track point (map to track point)
        // transformed_track_point = tf_buffer_->transform(track_point, map_->getFrameID(), tf2::durationFromSec(1.0));
        transformed_track_point = tf_buffer_->lookupTransform(map_->getFrameID(), track_point_frame_id_, tf2::TimePointZero);
    }
    catch(const tf2::TransformException& e)
    {
        RCLCPP_ERROR(this->get_logger(), "%s", e.what());
        return false;
    }
    RCLCPP_DEBUG(get_logger(), "Move to the position x: %f, y: %f", transformed_track_point.transform.translation.x, transformed_track_point.transform.translation.y);

    grid_map::Position position(transformed_track_point.transform.translation.x, transformed_track_point.transform.translation.y);
    // move map to the position
    map_->move(position);
    return true;
}   

bool ElevationMapping::updatePrediction(const rclcpp::Time& _time_stamp)
{
    if (!use_pose_update_) return true;
    if (time_tolerance_prediction_ + _time_stamp.seconds() < map_->getTimeOfLastUpdate().seconds())
    {
        RCLCPP_ERROR(get_logger(), "Requested update with time stamp %f, but time of last update was %f.", _time_stamp.seconds(), map_->getTimeOfLastUpdate().seconds());
        return false;
    }
    else if (_time_stamp < map_->getTimeOfLastUpdate(RCL_ROS_TIME))
    {
        RCLCPP_DEBUG(get_logger(), "Requested update with time stamp %f, but time of last update was %f. Ignore update", _time_stamp.seconds(), map_->getTimeOfLastUpdate().seconds());
        return true;
    }

    // Get robot pose at requested time.
    std::shared_ptr<geometry_msgs::msg::PoseWithCovarianceStamped const> pose_msg = pose_cache_.getElemBeforeTime(_time_stamp);
    if (!pose_msg)
    {
        // Tell the user that either for the timestamp no pose is available or that the buffer is possibly empty
        if (pose_cache_.getOldestTime() > last_point_cloud_update_time_)
        {
            RCLCPP_ERROR(get_logger(), "The oldest pose available is at %f, requested pose at %f", pose_cache_.getOldestTime().seconds(), last_point_cloud_update_time_.seconds());
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "Could not get pose from robot for time %f", last_point_cloud_update_time_.seconds());
        }
        return false;
    }
    Eigen::Matrix<double, 6, 6> pose_covariance = Eigen::Map<const Eigen::MatrixXd>(pose_msg->pose.covariance.data(), 6, 6);
    Eigen::Affine3d transform;
    tf2::fromMsg(pose_msg->pose.pose, transform);
    
    // compute map variacne update from motion prediction
    RCLCPP_DEBUG(get_logger(), "Calling RobotMotionUpdater::update");
    robot_motion_updater_->update(*map_, transform, pose_covariance, _time_stamp);

    RCLCPP_DEBUG(get_logger(), "Finished RobotMotionUpdater::update");
    return true;
}

void ElevationMapping::visibilityCleanUpCallback()
{
    map_->visibilityCleanup(last_point_cloud_update_time_);
}

bool ElevationMapping::readParameters()
{
    // elevation mapping
    use_pose_update_ = declare_parameter("use_pose_update", true);
    use_visibility_clean_up_ = declare_parameter("use_visibility_clean_up", true);
    visibility_clean_up_duration_ = declare_parameter("visibility_clean_up_duration", 1.0);
    use_and_publish_fused_map_ = declare_parameter("use_and_publish_fused_map", false);
    pose_cache_size_ = declare_parameter("pose_cache_size", 10);
    track_point_frame_id_ = declare_parameter("robot_frame", "/base_link");
    time_tolerance_prediction_ = declare_parameter("time_tolerance_prediction", 0.1); // second
    // extract_vaild_area_ = declare_parameter("extract_vaild_area", true);
    // declare_parameter("max_no_update_duration", max_no_update_duration_, 0.5);

    std::string map_frame, robot_frame;
    map_frame = declare_parameter("map_frame", "/map");
    robot_frame = track_point_frame_id_;

    for (size_t i=1; i > sensor_processors_.size(); ++i)
    {
        std::string prefix = "sensors.sensor"+std::to_string(i) + ".";
        std::string sensor_type = declare_parameter(prefix + "sensor_type", "");
        if (sensor_type == "perfect") 
        {
            auto common_config = getSensorProcessorConfig(get_node_logging_interface(), get_node_parameters_interface(), prefix);
            sensor_processors_.push_back(std::make_shared<PerfectSensorProcessor>(common_config, map_frame, robot_frame));
        }
        else if (sensor_type == "stereo") 
        {
            auto common_config = getSensorProcessorConfig(get_node_logging_interface(), get_node_parameters_interface(), prefix);
            auto stereo_config = getStereoSensorConfig(get_node_logging_interface(), get_node_parameters_interface(), prefix);
            sensor_processors_.push_back(std::make_shared<StereoSensorProcessor>(common_config, stereo_config, map_frame, robot_frame));
        }
        else if (sensor_type == "laser")
        {
            auto common_config = getSensorProcessorConfig(get_node_logging_interface(), get_node_parameters_interface(), prefix);
            auto laser_config = getLaserSensorConfig(get_node_logging_interface(), get_node_parameters_interface(), prefix);
            sensor_processors_.push_back(std::make_shared<LaserSensorProcessor>(common_config, laser_config, map_frame, robot_frame));
        }
        else break;
    }

    RCLCPP_DEBUG(get_logger(), "Constructed %lu sensor_processors", sensor_processors_.size());

    map_ = std::make_shared<ElevationMap>(getElevationMapConfig(get_node_logging_interface(), get_node_parameters_interface()), map_frame);

    return true;
}

}   

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(elevation_mapping::ElevationMapping)