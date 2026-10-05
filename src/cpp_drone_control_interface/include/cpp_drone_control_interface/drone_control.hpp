#pragma once

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include <geometry_msgs/msg/twist_stamped.hpp>
#include "cpp_velocity_controller/xyz_velocity_controller.hpp"

/**
 * @class DroneControl
 * @brief Interface to communicate with flight controller
 */
class DroneControl {
    public:
        /**
         * @class DroneControl::Result
         * @brief Result class for methods.
         */
        enum class Result { Success, Error, Running };

        virtual ~DroneControl() = default;

        /**
         * @return Current flight mode as string.
         */
        virtual std::string get_mode() = 0;

        /**
         * Publishes offboards target.
         * @param target Offboard target to be published.
         */
        virtual void set_target_position(const geometry_msgs::msg::PoseStamped& target) = 0;

        /**
         * Publishes offboard velocity
         * @param vx velocity on x-axis
         * @param vy velocity on y-axis
         * @param vz velocity on z-axis
         * @param yaw velocity on yaw-axis
         * @param yaw_rate yaw rate
         * @param ignore_yaw If true (standard), yaw and yaw_rate is ignored .
         */
        virtual void set_offboard_velocity(const double vx, const double vy, const double vz, const float yaw, const float yaw_rate, const bool ignore_yaw = true) = 0;

        /**
         * Publishes offboard Thrust
         * @param thrust Normalized Thrust (0.0 to 1.0)
         */
        virtual void set_offboard_thrust(const float thrust) = 0;

        /**
         * Switch to offboard
         * @return Success when mode is switched, Running while request is being made, and Error in case of Error.
         */
        virtual Result take_control() = 0;

        /**
         * @return True when mode is set to offboard, otherwise false.
         */
        virtual bool in_control() = 0;

        /**
         * Force arm
         * @return Success when drone is armed, Running while request is being made, and Error in case of Error.
         */
        virtual Result force_arm() = 0;

        /**
         * Force disarm
         * @return Success when drone is disarmed, Running while request is being made, and Error in case of Error.
         */
        virtual Result force_disarm() = 0;

        /**
         * @return current XyzVelocityController
         */
        virtual std::shared_ptr<XyzVelocityController> get_vel_controller() = 0;

        /**
         * Sets new XyzVelocityController
         * @param vel_ctrl XyzVelocityController to be used
         * @return Success when XyzVelocityController is set, and Error in case of Error.
         */
        virtual Result set_vel_controller(std::shared_ptr<XyzVelocityController> vel_ctrl) = 0;

        /**
         * @return true when flight controller state is "in-air"
         */
        virtual bool is_in_air() = 0; //checks flight controller state

        /**
        * The current position of the drone.
        * Caution: returns a nullptr if not position information is available!
        */
        virtual geometry_msgs::msg::PoseStamped::SharedPtr get_current_position() = 0;

        /**
        * The current twist (velocity) of the drone.
        * Caution: returns a nullptr if not position information is available!
        */
        virtual geometry_msgs::msg::TwistStamped::SharedPtr get_current_twist() = 0;

        virtual rclcpp::Logger get_logger() = 0; // for printing
        virtual rclcpp::Clock::SharedPtr get_clock() = 0;
};