#pragma once

#include <memory>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "command.hpp"
#include "cpp_service_client/magnet_service_client.hpp"
#include "offset_to_target.hpp"
#include "cpp_velocity_controller/xyz_velocity_controller.hpp"

class GoToPositionCommand : public Command {
 public:
  GoToPositionCommand(
    std::shared_ptr<DroneControl> drone_control,
    geometry_msgs::msg::PoseStamped target,
    const double max_speed = 0.02,
    const double position_reached_epsilon = 0.01f);

  Result exec();

 private:
  std::shared_ptr<DroneControl> drone_control_;
  geometry_msgs::msg::PoseStamped target_;
  const double max_speed_;
  const double position_reached_epsilon_;
};


class GotoCommand : public Command {
	public:
		GotoCommand(
            const std::shared_ptr<Command> command,
			const std::string name,
            const int step_on_complete = 1,
        	const int step_on_failure = 0,
        	const int step_on_running = 0,
        	const int step_on_error = 0,
        	const bool cancel_on_error = true,
        	const bool treat_failure_as_error = true,
            const bool is_return_statement = false);

		Result exec();

 	public:
        const std::shared_ptr<Command> command_;
   		const std::string name_;
   		const int step_on_complete_;
		const int step_on_failure_;
		const int step_on_running_;
    	const int step_on_error_;
    	const bool cancel_on_error_;
    	const bool treat_failure_as_error_;
        const bool is_return_statement_;
};

class CommandQueue : public Command {
	public:
		CommandQueue(
			std::shared_ptr<DroneControl> drone_control,
			std::string name,
            const bool reset_on_complete = false);

		Result exec();
		void add_command(std::shared_ptr<GotoCommand> command);

 private:
  std::vector<std::shared_ptr<GotoCommand>> command_sequence_;
  size_t current_command_ = 0;

 protected:
   std::shared_ptr<DroneControl> drone_control_;
   std::string name_;
   const bool reset_on_complete_;
};

/*
 ArmCommand added by Valentin Werner
 */
class ArmCommand : public Command {
public:
 ArmCommand(std::shared_ptr<DroneControl> drone_control, bool arm);

 Result exec();

private:
 std::shared_ptr<DroneControl> drone_control_;
 bool arm_;
};

/*
 MagnetCommand added by Valentin Werner
 */
class MagnetCommand : public Command {
public:
    MagnetCommand(
        std::shared_ptr<DroneControl> drone_control,
        std::shared_ptr<MagnetServiceClient> magnet_client,
        MagnetServiceClient::Direction direction);

 Result exec();
private:
 std::shared_ptr<DroneControl> drone_control_;
 std::shared_ptr<MagnetServiceClient> magnet_client_;
 MagnetServiceClient::Direction direction_;
};

/*
 IsInAirCommand added by Valentin Werner
 */
class IsInAirCommand : public Command {
public:
 IsInAirCommand(std::shared_ptr<DroneControl> drone_control);

 Result exec();

private:
 std::shared_ptr<DroneControl> drone_control_;
};


// base class for finding offset to tarket
class CalculateOffsetToTargetCommand : public Command {
public:
    CalculateOffsetToTargetCommand(
        std::shared_ptr<DroneControl> drone_control,
        std::shared_ptr<OffsetToTarget> offset_to_target);

    std::shared_ptr<OffsetToTarget> get_offset();
    virtual Result exec() = 0;

protected:
    std::shared_ptr<DroneControl> drone_control_;
    std::shared_ptr<OffsetToTarget> offset_to_target_;
};

class CalculateOffsetTf2Command : public CalculateOffsetToTargetCommand {
public:
    CalculateOffsetTf2Command(
        std::shared_ptr<DroneControl> drone_control,
        std::shared_ptr<tf2_ros::Buffer> tf_buffer,
        std::shared_ptr<tf2_ros::TransformListener> tf_listener,
        std::shared_ptr<OffsetToTarget> offset_to_target,
        const std::string target_frame,
        const double max_age_seconds = 1.0);

    Result exec();

private:
    const std::string target_frame_;
    const double max_age_seconds_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

class CalculateOffsetToGlobalPoseCommand : public CalculateOffsetToTargetCommand {
public:
    CalculateOffsetToGlobalPoseCommand(
        std::shared_ptr<DroneControl> drone_control,
        std::shared_ptr<OffsetToTarget> offset_to_target,
        std::shared_ptr<geometry_msgs::msg::PoseStamped> global_target_pose);

    Result exec() override;

private:
    std::shared_ptr<geometry_msgs::msg::PoseStamped> global_target_pose_;
};

class CreateGlobalTargetCommand : public Command {
public:
    CreateGlobalTargetCommand(
        std::shared_ptr<DroneControl> drone_control,
        std::shared_ptr<OffsetToTarget> offset,
        std::shared_ptr<geometry_msgs::msg::PoseStamped> global_pose);

    Result exec() override;

private:
    std::shared_ptr<DroneControl> drone_control_;
    std::shared_ptr<OffsetToTarget> offset_;
    std::shared_ptr<geometry_msgs::msg::PoseStamped> global_pose_;
};

class GoRelativToTargetCommand : public Command {
    public:
        enum class DesiredType {
            Ignore = 0,
            LessThanOrEqual,
            Equal,
            GreaterThanOrEqual,
        };

        GoRelativToTargetCommand(
            std::shared_ptr<DroneControl> drone_control,
            std::shared_ptr<CalculateOffsetToTargetCommand> targeter,
            std::shared_ptr<OffsetToTarget> desired_offset_to_target,
            const double desired_offset_reached_epsilon = 0.01,
            const double max_speed = 0.05,
            const DesiredType x_type = DesiredType::Ignore,
            const DesiredType y_type = DesiredType::Ignore,
            const DesiredType z_type = DesiredType::Ignore,
            const DesiredType yaw_type = DesiredType::Ignore,
            const DesiredType yaw_rate_type = DesiredType::Ignore,
            const bool use_vel_controller = true);

    Result exec();

private:
    std::shared_ptr<DroneControl> drone_control_;
    std::shared_ptr<CalculateOffsetToTargetCommand> targeter_;
    std::shared_ptr<OffsetToTarget> desired_offset_to_target_;
    const double desired_offset_reached_epsilon_;
    const double max_speed_;
    const DesiredType x_type_;
    const DesiredType y_type_;
    const DesiredType z_type_;
    const DesiredType yaw_type_;
    const DesiredType yaw_rate_type_;
    const bool use_vel_controller_;
};

class VelocityCommand : public Command {
public:
    VelocityCommand(std::shared_ptr<DroneControl> drone_control,
         std::shared_ptr<OffsetToTarget> desired_speed_to_target,
         const double max_speed,
         const double seconds = 0.f);

    Result exec();

private:
    std::shared_ptr<DroneControl> drone_control_;
    std::shared_ptr<OffsetToTarget> desired_speed_to_target_;
    const double max_speed_;
    const double seconds_;
    std::optional<rclcpp::Time> start_time_;
};

class ThrustCommand : public Command {
    public:
        ThrustCommand(std::shared_ptr<DroneControl> drone_control,
             const double thrust,
             const double seconds = 0.f);

    Result exec();

private:
    std::shared_ptr<DroneControl> drone_control_;
    const double thrust_;
    const double seconds_;
    std::optional<rclcpp::Time> start_time_;
};

class CenterBellowTargetCommandQueue : public CommandQueue {
public:
    CenterBellowTargetCommandQueue(
        std::shared_ptr<DroneControl> drone_control,
        std::shared_ptr<CalculateOffsetToTargetCommand> targeter,
        const double distance_below,
        const double max_speed,
        const bool reset_on_complete = false);

private:
    std::shared_ptr<CalculateOffsetToTargetCommand> targeter_;
    const double distance_below_;
    double max_speed_;
};

class MagnetDockAtTargetCommandQueue : public CommandQueue {
    public:
        MagnetDockAtTargetCommandQueue(
            std::shared_ptr<DroneControl> drone_control,
            std::shared_ptr<MagnetServiceClient> magnet_client,
            std::shared_ptr<CalculateOffsetToTargetCommand> targeter,
            const double max_speed,
          	const bool reset_on_complete = false);

    private:
        std::shared_ptr<MagnetServiceClient> magnet_client_;
        std::shared_ptr<CalculateOffsetToTargetCommand> targeter_;
        double max_speed_;
};

class MagnetTakedownCommandQueue : public CommandQueue {
    public:
        MagnetTakedownCommandQueue(
            std::shared_ptr<DroneControl> drone_control,
            std::shared_ptr<MagnetServiceClient> magnet_client,
            const double max_speed,
          	const bool reset_on_complete = false);

    private:
        std::shared_ptr<MagnetServiceClient> magnet_client_;
        double max_speed_;
};

class IsMovingCommand : public Command {
public:
    enum class Direction {
        Up,
        Down,
        Left,
        Right,
        Forward,
        Backward
    };

    IsMovingCommand(
        std::shared_ptr<DroneControl> drone_control,
        const Direction direction,
        const double moving_threshold = 0.015f);

     Result exec();

private:
    std::shared_ptr<DroneControl> drone_control_;
    const Direction direction_;
    const double moving_threshold_;

};

class OffboardCommand : public Command {
    public:
        OffboardCommand(
			std::shared_ptr<DroneControl> drone_control);

         Result exec();

    private:
		std::shared_ptr<DroneControl> drone_control_;

};