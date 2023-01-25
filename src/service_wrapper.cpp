/**
 * @file   service_wrapper.cpp
 * @author Dominik Authaler
 * @date   22.01.2022
 *
 * Utility class wrapping all the service related calls.
 */

#include "service_wrapper.hpp"
#include <chrono>

using namespace std::chrono_literals;

template <typename T>
bool checkServiceAvailability(T &serviceClient, std::chrono::seconds &timeout) {
    if (!serviceClient->wait_for_service(timeout) || !rclcpp::ok()) {
        return false;
    }

    return true;
}

ServiceWrapper::ServiceWrapper(bool ignoreDefaultParameters) : ignoreDefaultParameters(ignoreDefaultParameters) {
    node = rclcpp::Node::make_shared("rig_reconfigure");

    executor.add_node(node);

    thread = std::thread(&ServiceWrapper::threadFunc, this);
}

void ServiceWrapper::terminate() {
    terminateThread = true;

    // push 'empty' item to release thread from blocking wait
    requestQueue.push(std::make_shared<Request>(Request::Type::TERMINATE));

    executor.cancel();

    if (thread.joinable()) {
        thread.join();
    }
}

void ServiceWrapper::setNodeOfInterest(const std::string &name) {
    // TODO: cancel any pending service requests?

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

        switch (request->type) {
            case Request::Type::QUERY_NODE_NAMES: {
                auto response = std::make_shared<NodeNameResponse>(node->get_node_names());

                // ignore node used for querying the services and ros2cli daemon nodes
                auto it = std::remove_if(response->nodeNames.begin(), response->nodeNames.end(), [nodeName=std::string("/") + node->get_name()](const std::string &s) {
                    return (s == nodeName) || (s.find("/_ros2cli_daemon_") == 0);
                });

                response->nodeNames.erase(it, response->nodeNames.end());

                responseQueue.push(response);
                break;
            }

            case Request::Type::QUERY_NODE_PARAMETERS: {
                auto serviceRequest = std::make_shared<rcl_interfaces::srv::ListParameters::Request>();
                serviceRequest->depth = 0;

                auto result = listParametersClient->async_send_request(serviceRequest);

                if (executor.spin_until_future_complete(result) ==
                    rclcpp::FutureReturnCode::SUCCESS) {

                    auto valueRequest = std::make_shared<ParameterValueRequest>(result.get()->result.names);

                    if (ignoreDefaultParameters) {
                        // ignore node used for querying the services
                        auto it = std::remove_if(valueRequest->parameterNames.begin(), valueRequest->parameterNames.end(), [](const std::string &s) {
                            return (s.rfind("qos_overrides./", 0) == 0) || (s.rfind("use_sim_time", 0) == 0);
                        });

                        valueRequest->parameterNames.erase(it, valueRequest->parameterNames.end());
                    }

                    requestQueue.push(std::move(valueRequest));
                }
                break;
            }

            case Request::Type::QUERY_PARAMETER_VALUES: {
                auto valueRequest = std::dynamic_pointer_cast<ParameterValueRequest>(request);

                auto serviceRequest = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
                serviceRequest->names = valueRequest->parameterNames;

                auto result = getParametersClient->async_send_request(serviceRequest);

                if (executor.spin_until_future_complete(result) ==
                    rclcpp::FutureReturnCode::SUCCESS) {

                    auto response = std::make_shared<ParameterValueResponse>();

                    auto values = result.get()->values;
                    for (auto i = 0U; i < values.size(); i++) {
                        const auto &parameterName = valueRequest->parameterNames.at(i);
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

                    responseQueue.push(response);
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

                auto result = setParametersClient->async_send_request(update);

                if (executor.spin_until_future_complete(result) ==
                    rclcpp::FutureReturnCode::SUCCESS) {

                    auto resultMsg = result.get();

                    auto response = std::make_shared<ParameterModificationResponse>(resultMsg->results.at(0).successful, resultMsg->results.at(0).reason);
                    responseQueue.push(response);
                }
            }

            case Request::Type::TERMINATE:
                break;
        }

        executor.spin_once();
    }
}
