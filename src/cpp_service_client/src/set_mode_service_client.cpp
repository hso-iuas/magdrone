//
// Created by valwe on 2/14/26.
//

#include "cpp_service_client/set_mode_service_client.hpp"

SetModeServiceClient::SetModeServiceClient(rclcpp::Node* node) :
    ServiceClient<mavros_msgs::srv::SetMode>(node, "set_mode_service_client", "mavros/set_mode") {}

DroneControl::Result SetModeServiceClient::make_request(const std::string& mode) {
    if(!request_active_){
        auto request = this->instantiate_request();
        request->custom_mode = mode;
        return this->send_request(request);
    }

    return this->request_status();
}

bool SetModeServiceClient::check_success(std::shared_ptr<mavros_msgs::srv::SetMode::Response> response) {
    return response->mode_sent;
}
