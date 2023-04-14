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

/**
 * Small helper struct for tracking whether the task of a future has been finished in time.
 */
struct FutureTimeoutContainer {
    FutureTimeoutContainer() {
        timeSent = std::chrono::system_clock::now();
    }

    std::chrono::time_point<std::chrono::system_clock> timeSent;
    bool handled = false;
    bool timeoutReached = false;
};

class ServiceWrapper {
  public:
    explicit ServiceWrapper(bool ignoreDefaultParameters = true);

    void terminate();

    void setNodeOfInterest(const std::string &name);

    void pushRequest(RequestPtr &&request);

    ResponsePtr tryPopResponse();

    void checkForTimeouts();

  private:
    void threadFunc();
    void handleRequest(const RequestPtr &request);

    Queue<RequestPtr> requestQueue;
    Queue<ResponsePtr> responseQueue;

    std::atomic_bool terminateThread = false;

    bool ignoreDefaultParameters;
    std::string nodeName;
    std::thread thread;                   ///< Thread for offloading the creation of requests from the GPU thread.
    std::thread rosThread;                ///< Thread for spinning the ROS node.
    std::promise<bool> terminationHelper; ///< Helper future for terminating the ROS thread.

    std::mutex unfinishedROSRequestsMutex;
    std::vector<std::shared_ptr<FutureTimeoutContainer>> unfinishedROSRequests;

    std::shared_ptr<rclcpp::Node> node;
    rclcpp::executors::SingleThreadedExecutor executor;

    // clients for calling the different services
    rclcpp::Client<rcl_interfaces::srv::ListParameters>::SharedPtr listParametersClient;
    rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr getParametersClient;
    rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr setParametersClient;

    // callbacks for the results of the futures
    void nodeParametersReceived(rclcpp::Client<rcl_interfaces::srv::ListParameters>::SharedFuture future,
                                const std::shared_ptr<FutureTimeoutContainer> &timeoutContainer);
    void parameterValuesReceived(rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedFuture future,
                                 const std::vector<std::string> &parameterNames,
                                 const std::shared_ptr<FutureTimeoutContainer> &timeoutContainer);
    void parameterModificationResponseReceived(rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedFuture future,
                                               const std::string &parameterName,
                                               const std::shared_ptr<FutureTimeoutContainer> &timeoutContainer);
};

#endif // RIG_RECONFIGURE_SERVICE_WRAPPER_HPP
