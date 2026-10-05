#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include <mavros_msgs/msg/extended_state.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <mavros_msgs/msg/attitude_target.hpp>
#include <tf2/utils.h>

#include "cpp_service_client/arming_service_client.hpp"
#include "cpp_service_client/force_arming_service_client.hpp"
#include "cpp_service_client/set_mode_service_client.hpp"
#include "cpp_service_client/land_service_client.hpp"
#include "cpp_velocity_controller/pid_velocity_controller.hpp"
#include "cpp_drone_control_interface/drone_control.hpp"

#include <tf2/LinearMath/Matrix3x3.h>
#include <iostream>
#include <fstream>
#include "cpp_inifile/inicpp.h"

/**
 * @class MavrosDroneControl
 * @brief MAVROS Implementation of DroneControl to communicate with flight controller.
 */
class MavrosDroneControl : public DroneControl {
    public:
        MavrosDroneControl(rclcpp::Node* node);

        std::string get_mode();
        Result request_mode_change(const std::string& mode);

        void set_target_position(const geometry_msgs::msg::PoseStamped& target);
        void set_offboard_velocity(const double vx, const double vy, const double vz, const float yaw, const float yaw_rate, const bool ignore_yaw = true);
        void set_offboard_thrust(const float thrust);

        geometry_msgs::msg::PoseStamped::SharedPtr get_current_position();
        geometry_msgs::msg::TwistStamped::SharedPtr get_current_twist();

        rclcpp::Logger get_logger();
        rclcpp::Clock::SharedPtr get_clock();

        Result take_control();
        bool in_control();

        Result force_arm();
        Result force_disarm();

        bool is_in_air();
        std::shared_ptr<XyzVelocityController> get_vel_controller();
        Result set_vel_controller(std::shared_ptr<XyzVelocityController> vel_ctrl);

    private:

        void init_topics();
        void init_services();

        rclcpp::Node* node_;

        mavros_msgs::msg::State::SharedPtr mavros_state_ = nullptr;
        rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;

        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pos_pub_;

        geometry_msgs::msg::PoseStamped::SharedPtr current_position_ = nullptr;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pos_sub_;

        geometry_msgs::msg::TwistStamped::SharedPtr current_twist_ = nullptr;
        rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;

        mavros_msgs::msg::ExtendedState::SharedPtr ext_state_ = nullptr;
        rclcpp::Subscription<mavros_msgs::msg::ExtendedState>::SharedPtr ext_state_sub_;

        std::shared_ptr<ForceArmingServiceClient> force_arming_client_;
        std::shared_ptr<ArmingServiceClient> arming_client_;
        std::shared_ptr<SetModeServiceClient> set_mode_client_;

        std::shared_ptr<XyzVelocityController> vel_controller_ = nullptr;
        rclcpp::Publisher<mavros_msgs::msg::AttitudeTarget>::SharedPtr attitude_pub_;

        rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr velocity_pub_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};