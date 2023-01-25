/**
 * @file   service_wrapper.hpp
 * @author Dominik Authaler
 * @date   22.01.2022
 *
 * Utility class wrapping all the service related calls.
 */

#ifndef RIG_RECONFIGURE_SERVICE_WRAPPER_HPP
#define RIG_RECONFIGURE_SERVICE_WRAPPER_HPP

#include <vector>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include "queue.hpp"
#include "requests.hpp"
#include "responses.hpp"

class ServiceWrapper {
  public:
    ServiceWrapper(bool ignoreDefaultParameters = true);

    void terminate();

    void setNodeOfInterest(const std::string &name);

    void pushRequest(RequestPtr &&request);

    ResponsePtr tryPopResponse();

  private:
    void threadFunc();

    Queue<RequestPtr> requestQueue;
    Queue<ResponsePtr> responseQueue;

    std::atomic_bool terminateThread = false;

    bool ignoreDefaultParameters;
    std::string nodeName;
    std::thread thread;

    std::shared_ptr<rclcpp::Node> node;
    rclcpp::executors::SingleThreadedExecutor executor;

    // clients for calling the different services
    rclcpp::Client<rcl_interfaces::srv::ListParameters>::SharedPtr listParametersClient;
    rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr getParametersClient;
    rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr setParametersClient;
};

#endif // RIG_RECONFIGURE_SERVICE_WRAPPER_HPP
