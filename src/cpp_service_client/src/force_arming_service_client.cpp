//
// Created by valwe on 2/14/26.
//

#include "cpp_service_client/force_arming_service_client.hpp"

ForceArmingServiceClient::ForceArmingServiceClient(rclcpp::Node* node) :
    ServiceClient<mavros_msgs::srv::CommandLong>(node, "forced_arming_service_client", "/mavros/cmd/command") {}

DroneControl::Result ForceArmingServiceClient::make_request(bool arm){
    if(!request_active_){
        if(arm){
            RCLCPP_INFO(node_->get_logger(), "Arming...");
        }else{
            RCLCPP_INFO(node_->get_logger(), "Disarming...");
        }

        auto request = this->instantiate_request();
        request->command = mavros_msgs::msg::CommandCode::COMPONENT_ARM_DISARM;

        if(arm){
            RCLCPP_INFO(node_->get_logger(), "Arming...");
            request->param1 = 1.0;
        }else{
            RCLCPP_INFO(node_->get_logger(), "Disarming...");
            request->param1 = 0.0;
        }
        request->param2 = 21196.0; // 21196 = FORCE
        return this->send_request(request);
    }

    return this->request_status();
}