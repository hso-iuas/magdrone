//
// Created by valwe on 1/17/26.
//

#ifndef MAVROSCOMMANDER_H
#define MAVROSCOMMANDER_H


#pragma once

#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include "cpp_commands/commands.hpp"

#include "custom_services/srv/land_target.hpp"
#include "custom_services/srv/takedown.hpp"
#include "rclcpp/rclcpp.hpp"

/**
 * @class MavrosCommander
 * @brief Class to receive service calls and to run the main logic of this work.
 *
 * Executes the current command in the cmd_queue_ in a loop.
 * New commands are added by calling the services for attachment and detachment.
 */
class MavrosCommander {
    public:
        /**
         * @brief Constructs a new MavrosCommander.
         *
         * @param drone_control Interface to control the drone
         * @param magnet_client Client to controll magnet.
         * @param node ROS2 node this commander is attached to.
         */
        MavrosCommander(std::shared_ptr<DroneControl> drone_control,
                        std::shared_ptr<MagnetServiceClient> magnet_client,
                        rclcpp::Node* node);
        void command_loop();

    private:
        void init_services();
        std::shared_ptr<DroneControl> drone_control_;
        std::shared_ptr<MagnetServiceClient> magnet_client_;
        rclcpp::Node* node_;
        rclcpp::TimerBase::SharedPtr timer_;

        // internal command queue, to which service calls add commands.
        CommandQueue cmd_queue_;

        //tf2 buffers
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        // services
        rclcpp::Service<custom_services::srv::LandTarget>::SharedPtr land_target_service_;
        rclcpp::Service<custom_services::srv::Takedown>::SharedPtr takedown_service_;
};



#endif //MAVROSCOMMANDER_H
