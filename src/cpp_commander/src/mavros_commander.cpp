//
// Created by valwe on 1/17/26.
//

#include "mavros_commander.hpp"

using namespace std::chrono_literals;

MavrosCommander::MavrosCommander(std::shared_ptr<DroneControl> drone_control,
                                 std::shared_ptr<MagnetServiceClient> magnet_client,
                                 rclcpp::Node* node)

    : drone_control_(drone_control),
      magnet_client_(magnet_client),
      node_(node),
      cmd_queue_(drone_control_, "MavrosCommander") {

    init_services();

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(drone_control_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    timer_ = node->create_wall_timer(50ms, [this]() { this->command_loop(); });
}

void MavrosCommander::command_loop() {
    cmd_queue_.exec();
}

void MavrosCommander::init_services(){
    land_target_service_ = node_->create_service<custom_services::srv::LandTarget>(
        "magdrone/commander/land",
        [this](const std::shared_ptr<custom_services::srv::LandTarget::Request> request,
               std::shared_ptr<custom_services::srv::LandTarget::Response> response) {

            auto offset = std::make_shared<OffsetToTarget>(0, 0, 0, 0, 0);
            this->cmd_queue_.add_command(std::make_shared<GotoCommand>(
                std::make_shared<MagnetDockAtTargetCommandQueue>(
                      this->drone_control_,
                      this->magnet_client_,
                      std::make_shared<CalculateOffsetTf2Command>(
                          this->drone_control_,
                          tf_buffer_,
                          tf_listener_,
                          offset,
                          request->tf2_target,
						  3.0
                      ),
                      request->max_speed
                ),
                "MagnetDockAtTargetCommandQueue")
            );
            RCLCPP_INFO(node_->get_logger(), "MavrosCommander: added land_marker cmd");
            response->success = true;
        });

    takedown_service_ = node_->create_service<custom_services::srv::Takedown>(
        "magdrone/commander/takedown",
        [this](const std::shared_ptr<custom_services::srv::Takedown::Request> request,
               std::shared_ptr<custom_services::srv::Takedown::Response> response) {

            auto offset = std::make_shared<OffsetToTarget>(0, 0, 0, 0, 0);
            this->cmd_queue_.add_command(std::make_shared<GotoCommand>(
                std::make_shared<MagnetDetachCommandQueue>(
                      this->drone_control_,
                      this->magnet_client_,
                      request->max_speed
                ),
                "MagnetDetachCommandQueue")
            );
            RCLCPP_INFO(node_->get_logger(), "MavrosCommander: added takedown cmd");
            response->success = true;
        });

}