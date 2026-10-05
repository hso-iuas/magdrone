#ifndef SERVICE_HPP
#define SERVICE_HPP

#include <chrono>
#include <cstdlib>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "cpp_drone_control_interface/drone_control.hpp"

using namespace std::chrono_literals;

// for when there is no success field
template <typename T, typename = void>
struct has_success_member : std::false_type {};

// for when there is a success field
template <typename T>
struct has_success_member<T, std::void_t<decltype(std::declval<T>().success)>> : std::true_type {};



/**
 * @class ServiceClient
 * @brief Base class for ROS2 service calls.
 * ServiceType Template for ROS2 services
 */
template <typename ServiceType>
class ServiceClient {
    protected:
        rclcpp::Node* node_;
        bool request_active_ = false; //true if request is currently active

    private:
        typename rclcpp::Client<ServiceType>::SharedPtr client_;
        typename rclcpp::Client<ServiceType>::SharedFuture future_;
        std::string name_;
        std::string topic_;

    public:
        ServiceClient(rclcpp::Node* node, std::string name, std::string topic) : node_(node), name_(name), topic_(topic) {
            init();
        }

        bool is_request_active() {return request_active_;}

    protected:
        /**
         * Base init, to create ros2 service
         */
        void init(){
            RCLCPP_INFO(node_->get_logger(), std::string("Creating " + name_ + " client").c_str());

            client_ = node_->create_client<ServiceType>(topic_);

            while (!client_->wait_for_service(1s)) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(node_->get_logger(),
                             std::string("Interrupted while waiting for " + name_ +"-service. Exiting.").c_str());
                }
                RCLCPP_INFO(node_->get_logger(), std::string("Waiting for " + name_ + " service...").c_str());
            }
        }

        std::shared_ptr<typename ServiceType::Request> instantiate_request(){
            return std::make_shared<typename ServiceType::Request>();
        }

        DroneControl::Result send_request(std::shared_ptr<typename ServiceType::Request>  request){
            future_ = client_->async_send_request(request).share();
            request_active_ = true;
            return DroneControl::Result::Running;
        }

        /**
         * Method to check if request is successful, checks "success" field.
         * Services without this field shall override this method.
         * @param response Response to check success
         * @return True if successful
         */
        virtual bool check_success(std::shared_ptr<typename ServiceType::Response> response) {
            // If the response type has a 'success' field, compile this block:
            if constexpr (has_success_member<typename ServiceType::Response>::value) {
                return response->success;
            } else {
                // If it doesn't have a 'success' field, compile this block instead:
                RCLCPP_ERROR(node_->get_logger(),
                    "Response for %s has no 'success' field, but check_success() was not overridden!", name_.c_str());
                return false;
            }
        }

        /**
         * Checks status of request.
         * @return DroneControl::Result::Success when request was successful,
         * Running while no response was received yet,
         * Error when there is an error.
         */
        DroneControl::Result request_status(){
            // Waiting for a response to the request
            const bool received_response =
                (future_.wait_for(std::chrono::seconds(0)) ==
                std::future_status::ready);

            if (received_response) {
                auto msg = future_.get();
                if (check_success(msg)) {
                    RCLCPP_INFO(node_->get_logger(), std::string(name_ + " successful").c_str());
                    request_active_ = false;
                    return DroneControl::Result::Success;
                } else {
                    RCLCPP_INFO(node_->get_logger(), std::string(name_ + " denied").c_str());
                    return DroneControl::Result::Error;
                }
            } else {
                RCLCPP_INFO(node_->get_logger(), std::string("Waiting for response to " + name_ + " request").c_str());
                return DroneControl::Result::Running;
            }
        }
};

#endif //SERVICE_HPP
