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
#include <optional>

struct ROSParameter {
    ROSParameter(std::string name, std::variant<bool, int, double, std::string> value,
                 std::optional<std::size_t> searchPatternStart = std::nullopt,
                 std::optional<std::size_t> searchPatternEnd = std::nullopt) : name(std::move(name)), value(std::move(value)),
        searchPatternStart(searchPatternStart), searchPatternEnd(searchPatternEnd) {};

    std::string name;
    // TODO: add arrays
    std::variant<bool, int, double, std::string> value;

    // in case this parameter is part of a filtered parameter tree the following two members store the intermediate
    // information where in the name the applied search pattern is located
    std::optional<std::size_t> searchPatternStart;
    std::optional<std::size_t> searchPatternEnd;
};

#endif // RIG_RECONFIGURE_ROS_PARAMETER_HPP
