//
// Created by valwe on 2/14/26.
//

#ifndef FORCE_ARMING_SERVICE_CLIENT_HPP
#define FORCE_ARMING_SERVICE_CLIENT_HPP

#include "client_base.hpp"
#include "mavros_msgs/srv/command_long.hpp"
#include "mavros_msgs/msg/command_code.hpp"

/**
 * @class ForceArmingServiceClient
 * Implementation of ServiceClient to force arm/disarm drone
 */
class ForceArmingServiceClient : public ServiceClient<mavros_msgs::srv::CommandLong> {

public:
    ForceArmingServiceClient(rclcpp::Node* node);

    /**
     * Calls the service.
     * @param arm True = arm, False = Disarm
     * @return Success once the request is done, Running while waiting for the response and Error if an error accrues.
     */
    DroneControl::Result make_request(bool arm = true);
};


#endif //FORCE_ARMING_SERVICE_CLIENT_HPP
