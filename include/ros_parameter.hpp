/**
 * @file   ros_parameter.hpp
 * @author Dominik Authaler
 * @date   24.01.2023
 *
 * Utility class for representing a ROS parameter.
 */

#ifndef RIG_RECONFIGURE_ROS_PARAMETER_HPP
#define RIG_RECONFIGURE_ROS_PARAMETER_HPP

#include <string>
#include <variant>

#include <rcl_interfaces/msg/parameter_descriptor.hpp>

// TODO: add arrays
using ROSParameterVariant = std::variant<bool, int, double, std::string>;

struct ROSParameter {
    ROSParameter(std::string name_, ROSParameterVariant value_) :
        name(std::move(name_)), value(std::move(value_)) {};

    ROSParameter(rcl_interfaces::msg::ParameterDescriptor description_, ROSParameterVariant value_) :
        description(std::move(description_)), name(description.name), value(std::move(value_)) {};


    rcl_interfaces::msg::ParameterDescriptor description;
    std::string name;
    ROSParameterVariant value;
};

#endif // RIG_RECONFIGURE_ROS_PARAMETER_HPP
