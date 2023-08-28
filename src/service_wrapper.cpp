/**
 * @file   service_wrapper.cpp
 * @author Dominik Authaler
 * @date   22.01.2023
 *
 * Utility class wrapping all the service related calls.
 */

#include "service_wrapper.hpp"
#include <chrono>

using namespace std::chrono_literals;

constexpr auto ROS_SERVICE_TIMEOUT = 1s;

ServiceWrapper::ServiceWrapper(bool ignoreDefaultParameters_) : ignoreDefaultParameters(ignoreDefaultParameters_) {
    // according to https://design.ros2.org/articles/topic_and_service_names.html an underscore indicates
    // hidden nodes
    node = rclcpp::Node::make_shared("_rig_reconfigure");

    executor.add_node(node);

    thread = std::thread(&ServiceWrapper::threadFunc, this);
    rosThread = std::thread([&]() {
        while (!terminateThread) {
            executor.spin_until_future_complete(terminationHelper.get_future());
        }
    });
}

void ServiceWrapper::terminate() {
    terminateThread = true;
    terminationHelper.set_value(true);

    // push 'empty' item to release thread from blocking wait
    requestQueue.push(std::make_shared<Request>(Request::Type::TERMINATE));

    executor.cancel();

    if (thread.joinable()) {
        thread.join();
    }

    if (rosThread.joinable()) {
        rosThread.join();
    }
}

void ServiceWrapper::checkForTimeouts() {
    // check for timeouts
    auto curTime = std::chrono::system_clock::now();
    std::lock_guard unfinishedROSRequestsLock(unfinishedROSRequestsMutex);
    for (auto &unfinishedRequest : unfinishedROSRequests) {
        if ((curTime - unfinishedRequest->timeSent) > ROS_SERVICE_TIMEOUT) {
            responseQueue.push(std::make_shared<ServiceTimeout>(nodeName));
            unfinishedRequest->timeoutReached = true;
        }
    }

    // remove request if handled or timeout is reached
    std::erase_if(unfinishedROSRequests, [](const auto &request) {
        return request->handled || request->timeoutReached;
    });
}

void ServiceWrapper::setNodeOfInterest(const std::string &name) {
    if (listParametersClient) {
        listParametersClient->prune_pending_requests();
    }

    if (getParametersClient) {
        getParametersClient->prune_pending_requests();
    }

    if (setParametersClient) {
        setParametersClient->prune_pending_requests();
    }

    nodeName = name;

    listParametersClient = node->create_client<rcl_interfaces::srv::ListParameters>(nodeName + "/list_parameters");
    getParametersClient = node->create_client<rcl_interfaces::srv::GetParameters>(nodeName + "/get_parameters");
    setParametersClient = node->create_client<rcl_interfaces::srv::SetParameters>(nodeName + "/set_parameters");
}

void ServiceWrapper::pushRequest(RequestPtr &&request) {
    requestQueue.push(std::forward<RequestPtr>(request));
}

std::shared_ptr<Response> ServiceWrapper::tryPopResponse() {
    return responseQueue.try_pop().value_or(nullptr);
}

void ServiceWrapper::threadFunc() {
    while (!terminateThread) {
        auto request = requestQueue.pop();

        handleRequest(request);
    }
}

void ServiceWrapper::handleRequest(const RequestPtr &request) {
    switch (request->type) {
        case Request::Type::QUERY_NODE_NAMES: {
            // we are only interested in nodes which provide the parameter services and thus
            // query directly the service names (instead of the node names via get_node_names())
            const auto serviceMap = node->get_service_names_and_types();

            std::vector<std::string> nodeNames;
            for (const auto &[serviceName, _] : serviceMap) {
                // assumption: a node providing the list_parameters service hopefully also provides the services
                //             for requesting and modifying parameter values
                const auto idx = serviceName.find("/list_parameters");

                if (idx == std::string::npos) {
                    continue;
                }

                const std::string extractedNodeName = serviceName.substr(0, idx);

                // ignore 'hidden' nodes (like the ros2cli daemon nodes)
                if (extractedNodeName.starts_with("/_")) {
                    continue;
                }

                nodeNames.push_back(extractedNodeName);
            }

            // sort nodes alphabetically
            std::sort(nodeNames.begin(), nodeNames.end());

            auto response = std::make_shared<NodeNameResponse>(std::move(nodeNames));

            responseQueue.push(response);
            break;
        }

        case Request::Type::QUERY_NODE_PARAMETERS: {
            auto serviceRequest = std::make_shared<rcl_interfaces::srv::ListParameters::Request>();
            serviceRequest->depth = 0;

            const auto timeoutPtr = std::make_shared<FutureTimeoutContainer>();

            auto callbackLambda = [&, timeoutPtr](rclcpp::Client<rcl_interfaces::srv::ListParameters>::SharedFuture future) {
                std::lock_guard unfinishedROSRequestsLock(unfinishedROSRequestsMutex);
                nodeParametersReceived(std::forward<decltype(future)>(future), timeoutPtr);
            };

            listParametersClient->async_send_request(serviceRequest, callbackLambda);

            {
                std::lock_guard unfinishedROSRequestsLock(unfinishedROSRequestsMutex);
                unfinishedROSRequests.push_back(timeoutPtr);
            }
            break;
        }

        case Request::Type::QUERY_PARAMETER_VALUES: {
            auto valueRequest = std::dynamic_pointer_cast<ParameterValueRequest>(request);

            auto serviceRequest = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
            serviceRequest->names = valueRequest->parameterNames;

            const auto timeoutPtr = std::make_shared<FutureTimeoutContainer>();

            auto callbackLambda = [&, parameters=valueRequest->parameterNames, timeoutPtr](rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedFuture future) {
                std::lock_guard unfinishedROSRequestsLock(unfinishedROSRequestsMutex);
                parameterValuesReceived(std::forward<decltype(future)>(future), parameters, timeoutPtr);
            };

            getParametersClient->async_send_request(serviceRequest, callbackLambda);

            {
                std::lock_guard unfinishedROSRequestsLock(unfinishedROSRequestsMutex);
                unfinishedROSRequests.push_back(timeoutPtr);
            }
            break;
        }

        case Request::Type::MODIFY_PARAMETER_VALUE: {
            auto updateRequest = std::dynamic_pointer_cast<ParameterModificationRequest>(request);

            if (updateRequest->parameter.name.empty()) {
                break;
            }

            auto update = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
            rcl_interfaces::msg::Parameter parameterMsg;
            if (updateRequest->parameter.name.at(0) == '/') {
                parameterMsg.name = updateRequest->parameter.name.substr(1);
            } else {
                parameterMsg.name = updateRequest->parameter.name;
            }

            if (std::holds_alternative<int>(updateRequest->parameter.value)) {
                parameterMsg.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
                parameterMsg.value.integer_value = std::get<int>(updateRequest->parameter.value);
            } else if (std::holds_alternative<bool>(updateRequest->parameter.value)) {
                parameterMsg.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
                parameterMsg.value.bool_value = std::get<bool>(updateRequest->parameter.value);
            } else if (std::holds_alternative<double>(updateRequest->parameter.value)) {
                parameterMsg.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
                parameterMsg.value.double_value = std::get<double>(updateRequest->parameter.value);
            } else if (std::holds_alternative<std::string>(updateRequest->parameter.value)) {
                parameterMsg.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
                parameterMsg.value.string_value = std::get<std::string>(updateRequest->parameter.value);
            }

            update->parameters.push_back(parameterMsg);

            const auto timeoutPtr = std::make_shared<FutureTimeoutContainer>();

            auto callbackLambda = [&, name=parameterMsg.name, timeoutPtr](rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedFuture future) {
                parameterModificationResponseReceived(std::forward<decltype(future)>(future), name, timeoutPtr);
            };

            setParametersClient->async_send_request(update, callbackLambda);

            std::lock_guard unfinishedROSRequestsLock(unfinishedROSRequestsMutex);
            unfinishedROSRequests.push_back(timeoutPtr);
        }

        case Request::Type::TERMINATE:
            break;
    }
}

void ServiceWrapper::nodeParametersReceived(const rclcpp::Client<rcl_interfaces::srv::ListParameters>::SharedFuture &future,
                                            const std::shared_ptr<FutureTimeoutContainer> &timeoutContainer) {
    auto valueRequest = std::make_shared<ParameterValueRequest>(future.get()->result.names);

    if (ignoreDefaultParameters) {
        // ignore node used for querying the services
        std::erase_if(valueRequest->parameterNames, [](const std::string &s) {
            return (s.starts_with("qos_overrides./") || s.starts_with("use_sim_time"));
        });
    }

    timeoutContainer->handled = true;

    requestQueue.push(std::move(valueRequest));
}

void ServiceWrapper::parameterValuesReceived(const rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedFuture &future,
                                             const std::vector<std::string> &parameterNames,
                                             const std::shared_ptr<FutureTimeoutContainer> &timeoutContainer) {
    auto response = std::make_shared<ParameterValueResponse>();

    auto values = future.get()->values;
    for (auto i = 0U; i < values.size(); i++) {
        const auto &parameterName = parameterNames.at(i);
        const auto &valueMsg = values.at(i);
        switch (valueMsg.type) {
            case rcl_interfaces::msg::ParameterType::PARAMETER_BOOL:
                response->parameters.emplace_back(parameterName, valueMsg.bool_value);
                break;
            case rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER:
                response->parameters.emplace_back(parameterName, static_cast<int>(valueMsg.integer_value));
                break;
            case rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE:
                response->parameters.emplace_back(parameterName, valueMsg.double_value);
                break;
            case rcl_interfaces::msg::ParameterType::PARAMETER_STRING:
                response->parameters.emplace_back(parameterName, valueMsg.string_value);
                break;
            default:
                // arrays are currently not supported
                break;
        }
    }

    timeoutContainer->handled = true;

    responseQueue.push(response);
}

void ServiceWrapper::parameterModificationResponseReceived(const rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedFuture &future,
                                                           const std::string &parameterName,
                                                           const std::shared_ptr<FutureTimeoutContainer> &timeoutContainer) {
    const auto &resultMsg = future.get();

    auto response = std::make_shared<ParameterModificationResponse>(parameterName, resultMsg->results.at(0).successful, resultMsg->results.at(0).reason);

    timeoutContainer->handled = true;

    responseQueue.push(response);
}
