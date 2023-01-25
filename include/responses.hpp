/**
 * @file   responses.hpp
 * @author Dominik Authaler
 * @date   22.01.2022
 *
 * Possible responses which can be received from the service wrapper.
 */

#ifndef RIG_RECONFIGURE_RESPONSES_HPP
#define RIG_RECONFIGURE_RESPONSES_HPP

#include <utility>
#include <vector>
#include "ros_parameter.hpp"

struct Response {
    enum class Type {
        NODE_NAMES, PARAMETERS, MODIFICATION_RESULT
    };

    explicit Response(Type type) : type(type) {};
    virtual ~Response() = default;

    Type type;
};

using ResponsePtr = std::shared_ptr<Response>;

struct NodeNameResponse : public Response {
    explicit NodeNameResponse(const std::vector<std::string> &nodeNames) : Response(Type::NODE_NAMES), nodeNames(nodeNames) {};

    std::vector<std::string> nodeNames;
};

struct ParameterValueResponse : public Response {
    ParameterValueResponse() : Response(Type::PARAMETERS) {};

    std::vector<ROSParameter> parameters;
};

struct ParameterModificationResponse : public Response {
    ParameterModificationResponse(bool success, std::string reason) : Response(Type::MODIFICATION_RESULT), success(success), reason(std::move(reason)) {};

    bool success;
    std::string reason;
};

#endif // RIG_RECONFIGURE_RESPONSES_HPP
