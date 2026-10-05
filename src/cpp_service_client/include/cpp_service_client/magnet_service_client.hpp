//
// Created by valwe on 2/14/26.
//

#ifndef MAGNET_SERVICE_HPP
#define MAGNET_SERVICE_HPP

#include "client_base.hpp"
#include "custom_services/srv/drive_magnet.hpp"

/**
 * @class MagnetServiceClient
 * Implementation of ServiceClient to send commands to the magnet node.
 */
class MagnetServiceClient : private ServiceClient<custom_services::srv::DriveMagnet> {
    public:
    enum class Direction { Off, On, Down, Up };
    MagnetServiceClient(rclcpp::Node* node);

    /**
     * Calls the service.
     * @param direction Desired magnet behavior
     * @return Success once the request is done, Running while waiting for the response and Error if an error accrues.
     */
    DroneControl::Result make_request(Direction direction);
};



#endif //MAGNET_SERVICE_HPP
