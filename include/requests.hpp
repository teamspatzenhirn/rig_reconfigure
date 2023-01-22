/**
 * @file   requests.hpp
 * @author Dominik Authaler
 * @date   22.01.2022
 *
 * Possible requests which can be send to the service wrapper.
 */

#ifndef RIG_RECONFIGURE_REQUESTS_HPP
#define RIG_RECONFIGURE_REQUESTS_HPP

#include <vector>
#include <string>

struct Request {
    enum class Type {
        TERMINATE, QUERY_NODE_NAMES, QUERY_NODE_PARAMETERS, QUERY_PARAMETER_VALUES
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

#endif // RIG_RECONFIGURE_REQUESTS_HPP
