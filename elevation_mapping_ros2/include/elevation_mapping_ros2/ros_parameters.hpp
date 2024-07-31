#pragma once

#include <rclcpp/rclcpp.hpp>
#include <elevation_mapping_ros2/ElevationMap.hpp>
#include <elevation_mapping_ros2/sesnor_processing/SensorProcessorBase.hpp>
#include <elevation_mapping_ros2/sesnor_processing/StereoSensorProcessor.hpp>
#include <elevation_mapping_ros2/sesnor_processing/LaserSensorProcessor.hpp>

namespace elevation_mapping
{
inline bool getBoolParam(
    const std::string& param_name,
    const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr& node_logger, 
    const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr& node_params, 
    bool& value
)
{
    RCLCPP_INFO(node_logger->get_logger(), "Read %s", param_name.c_str());
    if (!node_params->has_parameter(param_name))
    {
        rcl_interfaces::msg::ParameterDescriptor desc;
        desc.name = param_name;
        desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
        desc.read_only = false;
        desc.dynamic_typing = true;

        node_params->declare_parameter(desc.name, rclcpp::ParameterValue(value), desc);

        rclcpp::Parameter param;
        const bool got_param = node_params->get_parameter(desc.name, param);

        if (!got_param) return false;
        if (rclcpp::PARAMETER_BOOL != param.get_type())
        {
            RCLCPP_FATAL(node_logger->get_logger(), "%s has wrong type", desc.name.c_str());
            return false;
        }

        value = param.get_parameter_value().get<bool>();
    }
    else 
    {
        RCLCPP_INFO(node_logger->get_logger(), "Read %s once more", param_name.c_str());
        rclcpp::Parameter param;
        const bool got_param = node_params->get_parameter(param_name, param);
        if (!got_param || rclcpp::PARAMETER_BOOL != param.get_type()) 
        {   
            RCLCPP_WARN(node_logger->get_logger(), "Failed to read %s", param_name.c_str());
            return false;
        }
        value = param.get_parameter_value().get<bool>();
    }

    return true;
}

inline bool getStringParam(
    const std::string& param_name,
    const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr& node_logger, 
    const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr& node_params, 
    std::string& value
)
{
    RCLCPP_INFO(node_logger->get_logger(), "Read %s", param_name.c_str());
    if (!node_params->has_parameter(param_name))
    {
        rcl_interfaces::msg::ParameterDescriptor desc;
        desc.name = param_name;
        desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
        desc.read_only = false;
        desc.dynamic_typing = true;

        node_params->declare_parameter(desc.name, rclcpp::ParameterValue(value), desc);

        rclcpp::Parameter param;
        const bool got_param = node_params->get_parameter(desc.name, param);

        if (!got_param) return false;
        if (rclcpp::PARAMETER_STRING != param.get_type())
        {
            RCLCPP_FATAL(node_logger->get_logger(), "%s has wrong type", desc.name.c_str());
            return false;
        }

        value = param.get_parameter_value().get<std::string>();
    }
    else {
        RCLCPP_INFO(node_logger->get_logger(), "Read %s once more", param_name.c_str());
        rclcpp::Parameter param;
        const bool got_param = node_params->get_parameter(param_name, param);
        if (!got_param || rclcpp::PARAMETER_STRING != param.get_type()) 
        {   
            RCLCPP_WARN(node_logger->get_logger(), "Failed to read %s", param_name.c_str());
            return false;
        }
        value = param.get_parameter_value().get<std::string>();
    }

    return true;
}

inline bool getIntParam(
    const std::string& param_name,
    const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr& node_logger, 
    const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr& node_params, 
    int& value
)
{
    RCLCPP_INFO(node_logger->get_logger(), "Read %s", param_name.c_str());
    if (!node_params->has_parameter(param_name))
    {
        rcl_interfaces::msg::ParameterDescriptor desc;
        desc.name = param_name;
        desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
        desc.read_only = false;
        desc.dynamic_typing = true;

        node_params->declare_parameter(desc.name, rclcpp::ParameterValue(value), desc);

        rclcpp::Parameter param;
        const bool got_param = node_params->get_parameter(desc.name, param);

        if (!got_param) return false;
        if (rclcpp::PARAMETER_INTEGER != param.get_type())
        {
            RCLCPP_FATAL(node_logger->get_logger(), "%s has wrong type", desc.name.c_str());
            return false;
        }

        value = param.get_parameter_value().get<int>();
    }
    else {
        RCLCPP_INFO(node_logger->get_logger(), "Read %s once more", param_name.c_str());
        rclcpp::Parameter param;
        const bool got_param = node_params->get_parameter(param_name, param);
        if (!got_param || rclcpp::PARAMETER_INTEGER != param.get_type()) 
        {   
            RCLCPP_WARN(node_logger->get_logger(), "Failed to read %s", param_name.c_str());
            return false;
        }
        value = param.get_parameter_value().get<int>();
    }
    return true;
}

inline bool getDoubleParam(
    const std::string& param_name,
    const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr& node_logger, 
    const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr& node_params, 
    double& value
)
{
    RCLCPP_INFO(node_logger->get_logger(), "Read %s", param_name.c_str());
    if (!node_params->has_parameter(param_name))
    {
        rcl_interfaces::msg::ParameterDescriptor desc;
        desc.name = param_name;
        desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
        desc.read_only = false;
        desc.dynamic_typing = true;
        node_params->declare_parameter(desc.name, rclcpp::ParameterValue(value), desc);

        rclcpp::Parameter param;
        const bool got_param = node_params->get_parameter(desc.name, param);

        if (!got_param) return false;
        if (rclcpp::PARAMETER_DOUBLE != param.get_type())
        {
            RCLCPP_FATAL(node_logger->get_logger(), "%s has wrong type", desc.name.c_str());
            return false;
        }
        value = param.get_parameter_value().get<double>();
    }
    else {
        RCLCPP_INFO(node_logger->get_logger(), "Read %s once more", param_name.c_str());
        rclcpp::Parameter param;
        const bool got_param = node_params->get_parameter(param_name, param);
        if (!got_param || rclcpp::PARAMETER_DOUBLE != param.get_type()) 
        {   
            RCLCPP_WARN(node_logger->get_logger(), "Failed to read %s", param_name.c_str());
            return false;
        }
        value = param.get_parameter_value().get<double>();
    }
    return true;
}

inline ElevationMap::Config getElevationMapConfig(
    const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr& node_logger, 
    const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr& node_params
)
{
    ElevationMap::Config config;
    std::string prefix = "elevation_map.";
    getStringParam(prefix+"logger_name", node_logger, node_params, config.logger_name_);
    getDoubleParam(prefix+"lateral_length", node_logger, node_params, config.lateral_length_);
    getDoubleParam(prefix+"longitudinal_length", node_logger, node_params, config.longitudinal_length_);
    getDoubleParam(prefix+"resolution", node_logger, node_params, config.resolution_);
    getDoubleParam(prefix+"initial_position_x", node_logger, node_params, config.initial_position_x_);
    getDoubleParam(prefix+"initial_position_y", node_logger, node_params, config.initial_position_y_);
    getDoubleParam(prefix+"min_normal_variance", node_logger, node_params, config.min_normal_variance_);
    getDoubleParam(prefix+"max_normal_variance", node_logger, node_params, config.max_nomral_variance_);
    getDoubleParam(prefix+"min_horizontal_variance", node_logger, node_params, config.min_horizontal_variance_);
    getDoubleParam(prefix+"max_horizontal_variance", node_logger, node_params, config.max_horizontal_variance_);
    getDoubleParam(prefix+"mahalanobis_distance_thres", node_logger, node_params, config.mahalanobis_distance_thres_);
    getDoubleParam(prefix+"scanning_duration", node_logger, node_params, config.scanning_duration_);
    getDoubleParam(prefix+"increase_height_alpha", node_logger, node_params, config.scanning_duration_);
    getDoubleParam(prefix+"multi_height_noise", node_logger, node_params, config.multi_height_noise_);

    return config;
}

inline SensorProcessorBase::CommonConfig getSensorProcessorConfig(
    const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr& node_logger, 
    const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr& node_params, 
    const std::string prefix
)
{
    SensorProcessorBase::CommonConfig config;
    getStringParam(prefix+"sensor_frame", node_logger, node_params, config.sensor_frame_);
    getBoolParam(prefix+"use_voxel_filter", node_logger, node_params, config.use_voxel_filter_);
    getDoubleParam(prefix+"voxel_leaf_size", node_logger, node_params, config.voxel_leaf_size_);
    getDoubleParam(prefix+"pass_filter_lower_threshold", node_logger, node_params, config.pass_filter_lower_threshold_);
    getDoubleParam(prefix+"pass_filter_upper_threshold", node_logger, node_params, config.pass_filter_upper_threshold_);

    return config;

}

inline StereoSensorProcessor::StereoSensorConfig getStereoSensorConfig(
    const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr& node_logger, 
    const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr& node_params, 
    const std::string prefix
)
{
    StereoSensorProcessor::StereoSensorConfig config;

    getDoubleParam(prefix+"depth_lower_limit", node_logger, node_params, config.depth_lower_limit_);
    getDoubleParam(prefix+"coef_lateral_variance", node_logger, node_params, config.coef_lateral_variance_);
    getDoubleParam(prefix+"coef_normal_variance", node_logger, node_params, config.coef_normal_variance_);
    getDoubleParam(prefix+"coef_lateral_variance", node_logger, node_params, config.coef_lateral_variance_);

    return config;
}

inline LaserSensorProcessor::LaserSensorConfig getLaserSensorConfig(
    const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr& node_logger, 
    const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr& node_params, 
    std::string prefix
)
{
    LaserSensorProcessor::LaserSensorConfig config;
    
    getDoubleParam(prefix + "min_radius", node_logger, node_params, config.min_radius_);
    getDoubleParam(prefix + "beam_angle", node_logger, node_params, config.beam_angle_);
    getDoubleParam(prefix + "beam_constant", node_logger, node_params, config.beam_constant_);

    return config;
}


}