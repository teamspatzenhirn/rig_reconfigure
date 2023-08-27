/**
 * @file   requests.hpp
 * @author Dominik Authaler
 * @date   22.01.2023
 *
 * Possible requests which can be send to the service wrapper.
 */

#ifndef RIG_RECONFIGURE_REQUESTS_HPP
#define RIG_RECONFIGURE_REQUESTS_HPP

#include <utility>
#include <vector>
#include <string>
#include "ros_parameter.hpp"

struct Request {
    enum class Type {
        TERMINATE, QUERY_NODE_NAMES, QUERY_NODE_PARAMETERS, QUERY_PARAMETER_VALUES, MODIFY_PARAMETER_VALUE
    };

    explicit Request(Type type_) : type(type_) {};
    virtual ~Request() = default;

    Type type;
};

using RequestPtr = std::shared_ptr<Request>;

struct ParameterValueRequest : Request {
    explicit ParameterValueRequest(const std::vector<std::string> &parameterNames_) : Request(Type::QUERY_PARAMETER_VALUES), parameterNames(parameterNames_) {};

    std::vector<std::string> parameterNames;
};

struct ParameterModificationRequest : Request {
    ParameterModificationRequest(ROSParameter updatedParameter_) : Request(Type::MODIFY_PARAMETER_VALUE), parameter(std::move(updatedParameter_)) {};

    ROSParameter parameter;
};

#endif // RIG_RECONFIGURE_REQUESTS_HPP
