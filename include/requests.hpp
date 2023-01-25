/**
 * @file   requests.hpp
 * @author Dominik Authaler
 * @date   22.01.2022
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

    explicit Request(Type type) : type(type) {};
    virtual ~Request() = default;

    Type type;
};

using RequestPtr = std::shared_ptr<Request>;

struct ParameterValueRequest : Request {
    explicit ParameterValueRequest(const std::vector<std::string> &parameterNames) : Request(Type::QUERY_PARAMETER_VALUES), parameterNames(parameterNames) {};

    std::vector<std::string> parameterNames;
};

struct ParameterModificationRequest : Request {
    ParameterModificationRequest(ROSParameter updatedParameter) : Request(Type::MODIFY_PARAMETER_VALUE), parameter(std::move(updatedParameter)) {};

    ROSParameter parameter;
};

#endif // RIG_RECONFIGURE_REQUESTS_HPP
