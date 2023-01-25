/**
 * @file   ros_parameter.hpp
 * @author Dominik Authaler
 * @date   24.01.2022
 *
 * Utility class for representing a ROS parameter.
 */

#ifndef RIG_RECONFIGURE_ROS_PARAMETER_HPP
#define RIG_RECONFIGURE_ROS_PARAMETER_HPP

#include <string>
#include <variant>

struct ROSParameter {
    ROSParameter(std::string name, std::variant<bool, int, double, std::string> value) : name(std::move(name)), value(std::move(value)) {};

    std::string name;
    // TODO: add arrays
    std::variant<bool, int, double, std::string> value;
};

#endif // RIG_RECONFIGURE_ROS_PARAMETER_HPP
