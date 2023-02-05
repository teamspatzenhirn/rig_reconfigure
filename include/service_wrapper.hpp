/**
 * @file   service_wrapper.hpp
 * @author Dominik Authaler
 * @date   22.01.2022
 *
 * Utility class wrapping all the service related calls.
 */

#ifndef RIG_RECONFIGURE_SERVICE_WRAPPER_HPP
#define RIG_RECONFIGURE_SERVICE_WRAPPER_HPP

#include <utility>
#include <vector>
#include <string>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include "queue.hpp"
#include "requests.hpp"
#include "responses.hpp"

using ListParametersFuture = rclcpp::Client<rcl_interfaces::srv::ListParameters>::FutureAndRequestId;
using GetParametersFuture = rclcpp::Client<rcl_interfaces::srv::GetParameters>::FutureAndRequestId;
using SetParametersFuture = rclcpp::Client<rcl_interfaces::srv::SetParameters>::FutureAndRequestId;
using ROSFutureVariant = std::variant<ListParametersFuture, GetParametersFuture, SetParametersFuture>;

struct ROSFuture {
    explicit ROSFuture(ROSFutureVariant future,
                       std::optional<std::vector<std::string>> requestedParameterNames = std::nullopt) : future(std::move(future)), requestedParameterNames(std::move(requestedParameterNames)) {
        timeSent = std::chrono::system_clock::now();
    }

    std::chrono::time_point<std::chrono::system_clock> timeSent;
    ROSFutureVariant future;
    bool handled = false;
    bool timeoutReached = false;

    std::optional<std::vector<std::string>> requestedParameterNames;
};

class ServiceWrapper {
  public:
    explicit ServiceWrapper(bool ignoreDefaultParameters = true);

    void terminate();

    void setNodeOfInterest(const std::string &name);

    void pushRequest(RequestPtr &&request);

    ResponsePtr tryPopResponse();

  private:
    void threadFunc();
    void handleRequest(const RequestPtr &request);

    Queue<RequestPtr> requestQueue;
    Queue<ResponsePtr> responseQueue;

    std::atomic_bool terminateThread = false;

    bool ignoreDefaultParameters;
    std::string nodeName;
    std::thread thread;

    std::vector<ROSFuture> unfinishedROSRequests;

    std::shared_ptr<rclcpp::Node> node;
    rclcpp::executors::SingleThreadedExecutor executor;

    // clients for calling the different services
    rclcpp::Client<rcl_interfaces::srv::ListParameters>::SharedPtr listParametersClient;
    rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr getParametersClient;
    rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr setParametersClient;
};

#endif // RIG_RECONFIGURE_SERVICE_WRAPPER_HPP
