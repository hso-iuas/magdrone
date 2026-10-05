//
// Created by valwe on 2/14/26.
//

#include "cpp_service_client/magnet_service_client.hpp"


MagnetServiceClient::MagnetServiceClient(rclcpp::Node* node) :
    ServiceClient<custom_services::srv::DriveMagnet>(node, "magent_service_client", "magdrone/magnet/drive") {}

DroneControl::Result MagnetServiceClient::make_request(Direction direction){
    if(!request_active_){
        auto request = this->instantiate_request();
        request->direction = (int)direction;

        return this->send_request(request);
    }

    return this->request_status();
}