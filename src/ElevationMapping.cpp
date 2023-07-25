#include <elevation_mapping_ros2/ElevationMapping.hpp>

namespace elevation_mapping {

using std::placeholders::_1;

ElevationMapping::ElevationMapping() : rclcpp::Node("elevation_mapping", rclcpp::NodeOptions().use_intra_process_comms(true))
{

    // create tf listener
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    // subscriber
    sub_point_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "input/point_cloud", 10, std::bind(&ElevationMapping::callbackPointcloud, this, _1)
    );

    pub_raw_map_ = this->create_publisher<grid_map_msgs::msg::GridMap>(
        "output/raw_map", 10
    );  

    if (use_pose_update_)
    {`
        sub_pose_.subscribe(this, "input/pose");
        pose_cache_.connectInput(sub_pose_);
        pose_cache_.setCacheSize(pose_cache_size_);
    }
    clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
}

void ElevationMapping::callbackPointcloud(const sensor_msgs::msg::PointCloud2::UniquePtr _point_cloud)
{
    PointCloudType::Ptr point_cloud;
    pcl::fromROSMsg(*_point_cloud, *point_cloud);
    
    last_point_cloud_update_time_(point_cloud->header.stamp*1000);

    Eigen::Matrix<double, 6, 6> robot_pose_covariance;
    robot_pose_covariance.setZero();
    if (use_pose_update_)
    {
        std::shared_ptr<geometry_msgs::msg::PoseWithCovarianceStamped const> pose_msg = pose_cache_.getElemBeforeTime(last_point_cloud_update_time_);
        if (!pose_msg) 
        {
            if (pose_cache_.getOldestTime().seconds() > last_point_cloud_update_time_.seconds())
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
    PointCloudType::Ptr point_cloud_map_frame(new PointCloudType);
    Eigen::VectorXf height_variance;
    if (!sensor_processor_->process(*point_cloud, robot_pose_covariance, point_cloud_map_frame, height_variance))
    {
        if (!sensor_processor_->isFirstTfAvailable())
        {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10, "Waiting for tf transformation to be available. (Message is throttled. 10s)");
            return;
        }
        else 
        {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 10, "Point cloud could not be processed. (Throttled 10s)");
            return;
        }
    }

    updateMapLocation();

    if (!updatePrediction(last_point_cloud_update_time_))
    {
        RCLCPP_ERROR(get_logger(), "Updating process noise failed");
        return ;
    }

    // add point cloud to elevation map
    if (!map_.add(point_cloud, height_variance, last_point_cloud_update_time_, sensor_processor_->getTransformSensor2Map()))
    {
        RCLCPP_ERROR(get_logger(), "Adding point cloud to elevation map failed");
        // reset map update timer
        return ;
    }    

    // fuse previous map and current map
    // map_.visibilityCleanup();
    // map_.fuseAll();

    grid_map_msgs::msg::GridMap message;
    grid_map::GridMapRosConverter::toMessage(map_.getRawMap(), message);
    pub_raw_map_->publish(message);
}

bool ElevationMapping::updateMapLocation()
{
    RCLCPP_DEBUG(this->get_logger(), "Elevation map is checked for relocalization");

    geometry_msgs::msg::TransformStamped track_point;
    track_point.header.frame_id = track_point_frame_id_;
    track_point.header.stamp = rclcpp::Time(0);
    track_point.transform.translation.x = 0.0;
    track_point.transform.translation.y = 0.0;
    track_point.transform.translation.z = 0.0;

    geometry_msgs::msg::TransformStamped transformed_track_point;
    try
    {
        // get transformed track point (map to track point)
        transformed_track_point = tf_buffer_->transform(track_point, map_.getFrameID(), rclcpp::Duration(1.0));
    }
    catch(const tf2::TransformException& e)
    {
        RCLCPP_ERROR(this->get_logger(), "%s", e.what());
        return false;
    }

    grid_map::Position position(transformed_track_point.transform.translation.x, transformed_track_point.transform.translation.y);
    // move map to the position
    map_.move(position);
    return true;
}   

bool ElevationMapping::updatePrediction(const rclcpp::Time& _time_stamp)
{
    if (_time_stamp + time_tolerance_ < map_.getTimeOfLastUpdate())
    {
        RCLCPP_ERROR(get_logger(), "Requested update with time stamp %f, but time of last update was %f.", _time_stamp.seconds(), map_.getTimeOfLastUpdate().seconds());
        return false;
    }
    else if (_time_stamp < map_.getTimeOfLastUpdate())
    {
        RCLCPP_DEBUG(get_logger(), "Requested update with time stamp %f, but time of last update was %f. Ignore update", _time_stamp.seconds(), map_.getTimeOfLastUpdate().seconds());
        return true;
    }

    // Get robot pose at requested time.
    std::shared_ptr<geometry_msgs::msg::PoseWithCovarianceStamped const> pose_msg = pose_cache_.getElemBeforeTime(_time_stamp);
    if (!pose_msg)
    {
        // Tell the user that either for the timestamp no pose is available or that the buffer is possibly empty
        if (pose_cache_.getOldestTime().seconds() > last_point_cloud_update_time_.seconds())
        {
            RCLCPP_ERROR(get_logger(), "The oldest pose available is at %f, requested pose at %f", pose_cache_.getOldestTime().seconds(), last_point_cloud_update_time_.seconds());
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "Could not get pose from robot for time %f", last_point_cloud_update_time_.seconds());
        }
        return false;
    }
    Eigen::Matrix<double, 6, 6> pose_covariance = Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>(pose_msg->pose.covariance.data(), 6, 6);
    Eigen::Affine3d transform;
    tf2::fromMsg(pose_msg, transform);
    
    // compute map variacne update from motion prediction
    robot_motion_updater_.update(map_, transform, pose_covariance, _time_stamp)

    return true;
}

bool ElevationMapping::readParameters()
{
    // elevation mapping
    declare_parameter("use_pose_update", use_pose_update_, true);
    declare_parameter("pose_cache_size", pose_cache_size_, 10);
    declare_parameter("track_point_frame_id", track_point_frame_id_, "/base_link");
    float time_tolerance; // miliseconds
    declare_parameter("time_tolerance_prediction", time_tolerance, 100);
    time_tolerance_prediction_(time_tolerance*1e6); // milli seconds -> nano seconds
    // declare_parameter("max_no_update_duration", max_no_update_duration_, 0.5);

    std::string sensor_type, sensor_frame, map_frame;
    declare_parameter("sensor_frame", sensor_frame, "/sensor");
    declare_parameter("map_frame", map_frame, "/map")
    declare_parameter("sensor_processor_type", sensor_type, "perfect");
    if (sensor_type == "perfect") sensor_processor_ = std::make_shared<PerfectSensorProcessor>(sensor_frame, map_frame, this->get_logger());
    else 
    {
        RCLCPP_ERROR(get_logger(), "The sensor type %s is invailed", sensor_type.c_str());
        return false;
    }

    map_.readParameter(this);
    sensor_processor_->readParam(this);


    return true;
}

}   