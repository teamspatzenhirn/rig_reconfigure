/**
* @file   responses.hpp
* @author Dominik Authaler
* @date   22.01.2022
*
* Possible responses which can be received from the service wrapper.
*/

#ifndef RIG_RECONFIGURE_RESPONSES_HPP
#define RIG_RECONFIGURE_RESPONSES_HPP

#include <vector>
#include <string>

struct Response {
    enum class Type {
        NODE_NAMES, PARAMETERS
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

#endif // RIG_RECONFIGURE_RESPONSES_HPP
