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
#include <vector>

// TODO: add arrays
struct DoubleArrayParam {
    bool isChanging;
    bool isChanged;
    std::vector<double> arrayValue;
};
using ROSParameterVariant = std::variant<bool, int, double, std::string, std::vector<double>, DoubleArrayParam>;

struct ROSParameter {
    ROSParameter(std::string name_, ROSParameterVariant value_) :
        name(std::move(name_)), value(std::move(value_)) {};

    std::string name;
    ROSParameterVariant value;
};

#endif // RIG_RECONFIGURE_ROS_PARAMETER_HPP
