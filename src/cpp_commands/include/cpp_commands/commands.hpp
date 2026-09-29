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

/**
 * @class GoToPositionCommand
 * @brief Guides the drone to a target.
 *
 * The target is considered reached once the center of gravity of the drone is closer than position_reached_epsilon to the target.
 */
class GoToPositionCommand : public Command {
    public:
        /**
         * @brief Constructs a new GoToPositionCommand.
         *
         * @param drone_control interface to control the drone
         * @param target target pose
         * @param max_speed maximal speed (m/s), default 0.02 m/s
         * @param position_reached_epsilon max accepted distance to target in meters.
         */
        GoToPositionCommand(
            std::shared_ptr<DroneControl> drone_control,
            geometry_msgs::msg::PoseStamped target,
            const double max_speed = 0.02,
            const double position_reached_epsilon = 0.01f
        );

        /**
         * @brief Executes the GoToPositionCommand logic.
         *
         * Calculates current offset and uses a VelocityCommand to command a velocity towards the target.
         *
         * @return Result::Complete once target is reached, Result::Running while navigating to the target and Result::Error in case of an error.
         */
        Result exec();

    private:
        std::shared_ptr<DroneControl> drone_control_;
        geometry_msgs::msg::PoseStamped target_;
        const double max_speed_;
        const double position_reached_epsilon_;
};

/**
 * @class GotoCommand
 * @brief Controls the execution flow by directing the sequence to a specific command. To be used with the CommandQueue Class.
 *
 * This command decorates another command and dictates the step offsets for the
 * execution sequence based on the underlying command's return state (Complete,
 * Failure, Running, or Error). It is used to build branching logic or loops within
 * command sequences.
 */
class GotoCommand : public Command {
	public:
        /**
         * @brief Constructs a GotoCommand with specific jump behavior inside of a CommandQueue.
         *
         * @param command The underlying command to evaluate or execute.
         * @param name The human-readable identifier for this command.
         * @param step_on_complete Steps to jump if the command completes successfully. Default is 1 (next step).
         * @param step_on_failure Steps to jump if the command fails. Default is 0 (stay on current).
         * @param step_on_running Steps to jump if the command is still running. Default is 0.
         * @param step_on_error Steps to jump if an error occurs. Default is 0.
         * @param cancel_on_error If true, cancels the entire sequence if an error occurs. Default is true.
         * @param treat_failure_as_error If true, a failure state is handled via the error pathway. Default is true.
         */
		GotoCommand(
            const std::shared_ptr<Command> command,
			const std::string name,
            const int step_on_complete = 1,
        	const int step_on_failure = 0,
        	const int step_on_running = 0,
        	const int step_on_error = 0,
        	const bool cancel_on_error = true,
        	const bool treat_failure_as_error = true);

         /**
         * @brief Runs exec() method of decorated command.
         *
         * @return Result exec() method of decorated command.
         */
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
};
/**
 * @class CommandQueue
 * @brief Sequence of GotoCommands, acting as one with jumps between them being handled.
 *
 * Executes commands in the queue, until the end of the sequence is reached, or an error is returned.
 * Next command to execute is based on the return value and the step configuration of the current command.
 */
class CommandQueue : public Command {
	public:
        /**
         * @brief Constructs a CommandQueue.
         *
         * @param drone_control The interface to control the drone.
         * @param name The human-readable identifier for this command.
         * @param reset_on_complete If true, the current_command_ will be reset to the start of the queue,
         * once the execution of the queue is successful.
         * By default this functionality is off (false).
         */
		CommandQueue(
			std::shared_ptr<DroneControl> drone_control,
			std::string name,
            const bool reset_on_complete = false);

        /**
         * @brief Runs exec() method of the current command.
         *
         * Runs current command in sequence.
         * Evaluates the return value of the current command.
         * Jumps the next command to be executed according to the jump information of the current GotoCommand.
         *
         * @return Result::Complete once execution of sequence finishes, Result::Running while still running the sequence and Result::Error in case of an error.
         */
		Result exec();

        /**
         * @brief Adds GotoCommand to the sequence
         */
		void add_command(std::shared_ptr<GotoCommand> command);

    private:
        std::vector<std::shared_ptr<GotoCommand>> command_sequence_; ///< Vector that represents the queue/sequence.
        size_t current_command_ = 0; ///< Index of current command .

    protected:
        std::shared_ptr<DroneControl> drone_control_;
        std::string name_;
        const bool reset_on_complete_;
};

/**
 * @class ArmCommand
 * @brief Force arm/disarm drone.
 *
 * Force is necessary in perching position.
 */
class ArmCommand : public Command {
    public:
        /**
         * @brief Constructs a CommandQueue.
         *
         * @param drone_control The interface to control the drone.
         * @param arm If true this command arms the drone, if false the drone is disarmed.
         */
        ArmCommand(std::shared_ptr<DroneControl> drone_control, bool arm);

        /**
         * @brief Arms/Disarms the drone.
         *
         * @return Result::Complete once arm/disarm is successful, Result::Running while waiting for drone to arm/disarm and Result::Error in case of an error.
         */
        Result exec();

    private:
        std::shared_ptr<DroneControl> drone_control_;
        bool arm_;
};

/**
 * @class MagnetCommand
 * @brief Actuates Magnet
 *
 * Moves the magnet, detachment and attachment position can be chosen.
 * It can also be moved one rotation up/down at a time.
 */
class MagnetCommand : public Command {
    public:
        /**
         * @brief Constructs a MagnetCommand.
         *
         * @param drone_control The interface to control the drone.
         * @param magnet_client The client that makes the request to the magnet service.
         * @param direction Desired magnet behavior: Detached/Attached/One Rotation Down/One Rotation Up
         */
        MagnetCommand(
            std::shared_ptr<DroneControl> drone_control,
            std::shared_ptr<MagnetServiceClient> magnet_client,
            MagnetServiceClient::Direction direction);

        /**
         * @brief Executes the magnet command
         * @return Result::Complete once magnet reached desired position, Running while moving, and Error in case of error.
         */
        Result exec();
    private:
        std::shared_ptr<DroneControl> drone_control_;
        std::shared_ptr<MagnetServiceClient> magnet_client_;
        MagnetServiceClient::Direction direction_;
};

/**
 * @class IsInAirCommand
 * @brief Checks if Flight Controller state is "In-Air".
 */
class IsInAirCommand : public Command {
    public:
        /**
         * @brief Constructs a IsInAirCommand.
         *
         * @param drone_control The interface to control the drone.
         */
        IsInAirCommand(std::shared_ptr<DroneControl> drone_control);

        /**
         * @brief Checks if Flight Controller state is "In-Air".
         * @return Result::Complete if In-Air and Result::Failure if state is NOT In-Air.
         */
        Result exec();

    private:
        std::shared_ptr<DroneControl> drone_control_;
};

/**
 * @class CalculateOffsetToTargetCommand
 * @brief Base class for finding a offset to a Target
 */
class CalculateOffsetToTargetCommand : public Command {
    public:
        /**
         * @brief Constructs a CalculateOffsetToTargetCommand.
         *
         * @param drone_control The interface to control the drone.
         * @param offset_to_target The in-out parameter that is be filled with the offset to a target.
         */
        CalculateOffsetToTargetCommand(
            std::shared_ptr<DroneControl> drone_control,
            std::shared_ptr<OffsetToTarget> offset_to_target);

        /**
         * @brief Returns current offset to target.
         */
        std::shared_ptr<OffsetToTarget> get_offset();
        virtual Result exec() = 0;

    protected:
        std::shared_ptr<DroneControl> drone_control_;
        std::shared_ptr<OffsetToTarget> offset_to_target_;
};

/**
 * @class CalculateOffsetTf2Command
 * @brief Finds offset from provided TF2 target to the magnet
 *
 * For x- and y-axis the offset is calculated from the center of the drone.
 * For z-axis the offset is calculated from the tip of the magnet.
 */
class CalculateOffsetTf2Command : public CalculateOffsetToTargetCommand {
    public:
        /**
         * @brief Constructs a CalculateOffsetTf2Command.
         *
         * @param drone_control The interface to control the drone.
         * @param tf_buffer TF2 buffer.
         * @param tf_listener Tf2 listener.
         * @param offset_to_target The in-out parameter that is be filled with the offset to a target.
         * @param target_frame The target TF2 frame.
         * @param max_age_seconds Maximal age in seconds. If last transformation is older than this value, the exec method returns Failure.
         */
        CalculateOffsetTf2Command(
            std::shared_ptr<DroneControl> drone_control,
            std::shared_ptr<tf2_ros::Buffer> tf_buffer,
            std::shared_ptr<tf2_ros::TransformListener> tf_listener,
            std::shared_ptr<OffsetToTarget> offset_to_target,
            const std::string target_frame,
            const double max_age_seconds = 1.0);

        /**
         * @brief Calculates current offset to target.
         *
         * Returns Failure when transformation is older than max_age_seconds_.
         */
        Result exec();

    private:
        const std::string target_frame_;
        const double max_age_seconds_;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

/**
 * @class CalculateOffsetToGlobalPoseCommand
 * @brief Finds offset from provided global target.
*/
class CalculateOffsetToGlobalPoseCommand : public CalculateOffsetToTargetCommand {
    public:
        /**
         * @brief Constructs a CalculateOffsetToGlobalPoseCommand.
         *
         * @param drone_control The interface to control the drone.
         * @param offset_to_target The in-out parameter that is be filled with the offset to a target.
         * @param global_target_pose The target pose.
         */
        CalculateOffsetToGlobalPoseCommand(
            std::shared_ptr<DroneControl> drone_control,
            std::shared_ptr<OffsetToTarget> offset_to_target,
            std::shared_ptr<geometry_msgs::msg::PoseStamped> global_target_pose);

        /**
         * @brief Calculates current offset to target.
         */
        Result exec() override;

    private:
        std::shared_ptr<geometry_msgs::msg::PoseStamped> global_target_pose_;
};

/**
 * @class CreateGlobalTargetCommand
 * @brief Creates a global target from an offset.
*/
class CreateGlobalTargetCommand : public Command {
    public:
        /**
         * @brief Constructs a CreateGlobalTargetCommand.
         *
         * @param drone_control The interface to control the drone.
         * @param offset The offset from the current pose.
         * @param global_pose In-out parameter that will be filled with the created global target.
         */
        CreateGlobalTargetCommand(
            std::shared_ptr<DroneControl> drone_control,
            std::shared_ptr<OffsetToTarget> offset,
            std::shared_ptr<geometry_msgs::msg::PoseStamped> global_pose);

        /**
         * @brief Calculates global pose with offset from current pose.
         *
         * Populates global_pose.
         */
        Result exec() override;

    private:
        std::shared_ptr<DroneControl> drone_control_;
        std::shared_ptr<OffsetToTarget> offset_;
        std::shared_ptr<geometry_msgs::msg::PoseStamped> global_pose_;
};

/**
 * @class GoRelativeToTargetCommand
 * @brief Navigates the drone to the target using a targeter.
 *
 * The axis to guide towards the target can be chosen.
 * As can the condition by with an axis is considered to have reached its target.
 */
class GoRelativeToTargetCommand : public Command {
    public:
        /**
         * @class DesiredType
         * @brief How to consider if a target is reached.
         *
         * Ignored = Axis is ignored.
         * LessThanOrEqual = Target is reached when offset is less of equal to desired offset.
         * Equal = Axis is eqaul to desired offset.
         * GreaterThanOrEqual = Target is reached when offset is greater or equal to the desired offset.
         */
        enum class DesiredType {
            Ignore = 0,
            LessThanOrEqual,
            Equal,
            GreaterThanOrEqual,
        };

        /**
         * @brief Constructs a GoRelativeToTargetCommand.
         *
         * @param drone_control The interface to control the drone.
         * @param targeter The targeter used to find he offset to the target.
         * @param desired_offset_to_target Desired offset to the target.
         * @param desired_offset_reached_epsilon Epsilon of target reached.
         * @param max_speed Max speed to use for approach.
         * @param x_type DesiredType for x-axis.
         * @param y_type DesiredType for y-axis.
         * @param z_type DesiredType for z-axis.
         * @param yaw_type DesiredType for yaw.
         * @param yaw_rate_type DesiredType for yaw-rate.
         * @param use_vel_controller True when velocity controller shall be used (default).
         * False when distance from target shall be used as velocity towards the target.
         */
        GoRelativeToTargetCommand(
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

    /**
     * @brief Navigates drone to target
     *
     * Uses targeter_ to get current offset to target.
     * Returns Failure if targeter_ returns failure.
     */
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

/**
 * @class VelocityCommand
 * @brief Commands velocity.
 */
class VelocityCommand : public Command {
    public:
        /**
         * @brief Constructs a VelocityCommand.
         *
         * @param drone_control The interface to control the drone.
         * @param desired_speed_to_target Velocity to command.
         * @param max_speed Max speed to use.
         * @param seconds Time after which the command returns complete (default is 0 = command velocity once)
         */
        VelocityCommand(std::shared_ptr<DroneControl> drone_control,
             std::shared_ptr<OffsetToTarget> desired_speed_to_target,
             const double max_speed,
             const double seconds = 0.f);

        /**
         * @brief Commands the velocity.
         */
        Result exec();

    private:
        std::shared_ptr<DroneControl> drone_control_;
        std::shared_ptr<OffsetToTarget> desired_speed_to_target_;
        const double max_speed_;
        const double seconds_;
        std::optional<rclcpp::Time> start_time_; ///< start time for the duration functionality
};

/**
 * @class ThrustCommand
 * @brief Commands thrust.
 */
class ThrustCommand : public Command {
    public:
        /**
         * @brief Constructs a ThrustCommand.
         *
         * @param drone_control The interface to control the drone.
         * @param thrust Thrust to command.
         * @param seconds Time after which the command returns complete (default is 0 = command thrust once)
         */
        ThrustCommand(std::shared_ptr<DroneControl> drone_control,
             const double thrust,
             const double seconds = 0.f);

    /**
     * @brief Commands the thrust.
     */
    Result exec();

    private:
        std::shared_ptr<DroneControl> drone_control_;
        const double thrust_;
        const double seconds_;
        std::optional<rclcpp::Time> start_time_;
};

/**
 * @class CenterBellowTargetCommandQueue
 * @brief Centers drone at choosen distance bellow traget.
 *
 */
class CenterBellowTargetCommandQueue : public CommandQueue {
public:
    /**
     * @brief Constructs a CenterBellowTargetCommandQueue.
     *
     * @param drone_control The interface to control the drone.
     * @param targeter Targeter to use for offset calculation..
     * @param distance_below Desired distance bellow the target.
     * @param max_speed Max speed for the approach.
     * @param reset_on_complete If command queue shall be reset once executions finishes, default is false.
     */
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

/**
 * @class MagnetDockAtTargetCommandQueue
 * @brief Docks drone magnetically to target.
 */
class MagnetDockAtTargetCommandQueue : public CommandQueue {
    public:
        /**
         * @brief Constructs a MagnetDockAtTargetCommandQueue.
         *
         * @param drone_control The interface to control the drone.
         * @param magnet_client Client to control the magnet.
         * @param targeter Targeter to use for offset calculation.
         * @param max_speed Max speed for the approach.
         * @param reset_on_complete If command queue shall be reset once executions finishes, default is false.
         */
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

/**
 * @class MagnetDetachCommandQueue
 * @brief Detaches magnet and safely pilots drone from the perching position.
 */
class MagnetDetachCommandQueue : public CommandQueue {
    public:
        /**
         * @brief Constructs a MagnetDetachCommandQueue.
         *
         * @param drone_control The interface to control the drone.
         * @param magnet_client Client to control the magnet.
         * @param max_speed Max speed for the detachment.
         * @param reset_on_complete If command queue shall be reset once executions finishes, default is false.
         */
        MagnetDetachCommandQueue(
            std::shared_ptr<DroneControl> drone_control,
            std::shared_ptr<MagnetServiceClient> magnet_client,
            const double max_speed,
          	const bool reset_on_complete = false);

    private:
        std::shared_ptr<MagnetServiceClient> magnet_client_;
        double max_speed_;
};

/**
 * @class IsMovingCommand
 * @brief Command to check if drone is moving in a given IsMovingCommand::Direction.
 */
class IsMovingCommand : public Command {
public:
    /**
     * @class Direction
     * @brief Direction in which to check movement.
     */
    enum class Direction {
        Up,
        Down,
        Left,
        Right,
        Forward,
        Backward
    };

    /**
     * @brief Constructs a IsMovingCommand.
     *
     * @param drone_control The interface to control the drone.
     * @param direction Direction to check.
     * @param moving_threshold Velocity threshold from which the drone is considered moving.
     */
    IsMovingCommand(
        std::shared_ptr<DroneControl> drone_control,
        const Direction direction,
        const double moving_threshold = 0.015f);

    /**
     * @brief Checks if drone is moving.
     *
     * Returns Complete if drone is moving in the selected direction.
     * Returns Failure if drone is not moving in the selected direction.
     */
    Result exec();

private:
    std::shared_ptr<DroneControl> drone_control_;
    const Direction direction_;
    const double moving_threshold_;

};

/**
 * @class OffboardCommand
 * @brief Switches drone to offboard mode.
 */
class OffboardCommand : public Command {
    public:
        /**
         * @brief Constructs a OffboardCommand.
         *
         * @param drone_control The interface to control the drone.
         */
        OffboardCommand(
			std::shared_ptr<DroneControl> drone_control);

        /**
         * @brief Switches drone to offboard mode.
         *
         * Returns Complete when mode is switched.
         * Returns Running when mode is NOT switched yet.
         * Returns Error in case of Error.
         */
        Result exec();

    private:
		std::shared_ptr<DroneControl> drone_control_;
};