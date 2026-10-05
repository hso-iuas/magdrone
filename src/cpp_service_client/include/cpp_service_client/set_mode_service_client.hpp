//
// Created by valwe on 2/14/26.
//

#ifndef SET_MODE_SERVICE_CLIENT_HPP
#define SET_MODE_SERVICE_CLIENT_HPP


#include "client_base.hpp"
#include "mavros_msgs/srv/set_mode.hpp"

/**
 * @class SetModeServiceClient
 * Implementation of ServiceClient to set flight mode
 */
class SetModeServiceClient : public ServiceClient<mavros_msgs::srv::SetMode> {

public:
    SetModeServiceClient(rclcpp::Node* node);

    /**
     * Calls the service.
     * @param mode Desired flight mode
     * @return Success once the request is done, Running while waiting for the response and Error if an error accrues.
     */
    DroneControl::Result make_request(const std::string& mode);

    /**
     * Override as service does not return a success field in response.
     * @param response Response to check
     * @return True if mode was set successfully
     */
    bool check_success(std::shared_ptr<mavros_msgs::srv::SetMode::Response> response) override;
};

#endif //SET_MODE_SERVICE_CLIENT_HPP
