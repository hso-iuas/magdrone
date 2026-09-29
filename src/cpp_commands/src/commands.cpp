#include <cmath>

#include "cpp_commands/commands.hpp"

GoToPositionCommand::GoToPositionCommand(
    std::shared_ptr<DroneControl> drone_control,
    geometry_msgs::msg::PoseStamped target,
    const double max_speed,
    const double position_reached_epsilon)
		: drone_control_(drone_control),
      	target_(target), max_speed_(max_speed),
      	position_reached_epsilon_(position_reached_epsilon) {}

Command::Result GoToPositionCommand::exec() {

	// in control?
  	if(!drone_control_->in_control()) {
     	return Result::Error;
  	}

	auto current_position = drone_control_->get_current_position();
  	if (current_position == nullptr) {
    	RCLCPP_INFO(drone_control_->get_logger(),
                "GoToPositionCommand: Waiting for position info");
    	return Result::Running;
  	}

    const double dx = target_.pose.position.x - current_position->pose.position.x;
    const double dy = target_.pose.position.y - current_position->pose.position.y;
    const double dz = target_.pose.position.z - current_position->pose.position.z;

    const double distance_to_target = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2) + std::pow(dz, 2));

    const double vx = std::min(max_speed_, std::abs(dx) / distance_to_target * max_speed_) * (dx > 0 ? 1 : -1);
    const double vy = std::min(max_speed_, std::abs(dy) / distance_to_target * max_speed_) * (dy > 0 ? 1 : -1);
	const double vz = std::min(max_speed_, std::abs(dz) / distance_to_target * max_speed_) * (dz > 0 ? 1 : -1);

  	RCLCPP_INFO(drone_control_->get_logger(),
		"GoToPositionCommand: exec, dist=%f d(%f, %f, %f) v(%f, %f, %f)",
		distance_to_target,
        dx,dy,dz,
        vx, vy, vz);

    VelocityCommand v_cmd(drone_control_, std::make_shared<OffsetToTarget>(vx, vy, vz, 0, 0), max_speed_);
    v_cmd.exec();

  	if (distance_to_target > position_reached_epsilon_) {
    	return Result::Running;
  	} else {
    	RCLCPP_INFO(drone_control_->get_logger(),
            "GoToPositionCommand: Reached target");
    	return Result::Complete;
  	}
}

GotoCommand::GotoCommand(
	const std::shared_ptr<Command> command,
	const std::string name,
    const int step_on_complete,
    const int step_on_failure,
    const int step_on_running,
    const int step_on_error,
    const bool cancel_on_error,
    const bool treat_failure_as_error)
    	: command_(command),
		name_(name),
        step_on_complete_(step_on_complete),
        step_on_failure_(step_on_failure),
        step_on_running_(step_on_running),
        step_on_error_(step_on_error),
        cancel_on_error_(cancel_on_error),
        treat_failure_as_error_(treat_failure_as_error) {}

Command::Result GotoCommand::exec(){
	return command_->exec();
}

CommandQueue::CommandQueue(
    std::shared_ptr<DroneControl> drone_control,
	std::string name,
	const bool reset_on_complete)
    	: drone_control_(drone_control), name_(name), reset_on_complete_(reset_on_complete) {}

Command::Result CommandQueue::exec(){
	if (current_command_ >= command_sequence_.size()) {
        if (reset_on_complete_){current_command_ = 0;}
		return Command::Result::Complete;
	}
    auto result = command_sequence_[current_command_]->exec();

    switch (result) {
      	case Command::Result::Complete:
      		RCLCPP_WARN(drone_control_->get_logger(),
						"%s - CommandQueue: command %zu done",
						name_.c_str(),
						current_command_);
    		current_command_ += command_sequence_[current_command_]->step_on_complete_;
            return Command::Result::Running;
	        break;
        case Command::Result::Failure:
        	RCLCPP_WARN(drone_control_->get_logger(),
				"%s - CommandQueue: command %zu failure, jumping",
				name_.c_str(),
				current_command_);
    		if(!command_sequence_[current_command_]->treat_failure_as_error_){
    			current_command_ += command_sequence_[current_command_]->step_on_failure_;
                return Command::Result::Running;
                break;
    		}
    		// fall through to treat as error

            RCLCPP_WARN(drone_control_->get_logger(),
				"%s - CommandQueue: command %zu failure, !fallthrough!",
				name_.c_str(),
				current_command_);
            [[fallthrough]];
     	case Command::Result::Error:
     		RCLCPP_ERROR(drone_control_->get_logger(),
				 "%s - CommandQueue: command %zu returned with error",
				 name_.c_str(),
				 current_command_);
    		if(command_sequence_[current_command_]->cancel_on_error_){
    			current_command_ = command_sequence_.size();
                return Command::Result::Error;
    		}else{
    			current_command_ += command_sequence_[current_command_]->step_on_error_;
                return Command::Result::Running;
    		}
            break;
        case Command::Result::Running:
         	current_command_ += command_sequence_[current_command_]->step_on_running_;
            return Command::Result::Running;
            break;
		default:
			RCLCPP_ERROR(drone_control_->get_logger(), "At 'unreachable' code :(");
            return Command::Result::Error;
            break;
    }
}

void CommandQueue::add_command(std::shared_ptr<GotoCommand> command){
	command_sequence_.emplace_back(command);
}

ArmCommand::ArmCommand(
    std::shared_ptr<DroneControl> drone_control,
    bool arm)
    	: drone_control_(drone_control), arm_(arm) {}

Command::Result ArmCommand::exec(){
  DroneControl::Result result;

  if(arm_){
    result = drone_control_->force_arm();
  } else {
    result = drone_control_->force_disarm();
  }

  if(result == DroneControl::Result::Success){return Command::Result::Complete;}
  else if(result == DroneControl::Result::Error){return Command::Result::Error;}
  else {return Command::Result::Running;}
}

MagnetCommand::MagnetCommand(
    std::shared_ptr<DroneControl> drone_control,
    std::shared_ptr<MagnetServiceClient> magnet_client,
    MagnetServiceClient::Direction direction)
		: drone_control_(drone_control), magnet_client_(magnet_client), direction_(direction) {}

Command::Result MagnetCommand::exec(){
  DroneControl::Result result = magnet_client_->make_request(direction_);

  if(result == DroneControl::Result::Success){return Command::Result::Complete;}
  else if(result == DroneControl::Result::Error){return Command::Result::Error;}
  else {return Command::Result::Running;}
}

IsInAirCommand::IsInAirCommand(
	std::shared_ptr<DroneControl> drone_control)
		: drone_control_(drone_control) {}

Command::Result IsInAirCommand::exec(){
    if(drone_control_->is_in_air()) {return Result::Complete;}
    return Result::Failure;
}

CalculateOffsetToTargetCommand::CalculateOffsetToTargetCommand(
    std::shared_ptr<DroneControl> drone_control,
    std::shared_ptr<OffsetToTarget> offset_to_target)
		: drone_control_(drone_control), offset_to_target_(offset_to_target) {}

std::shared_ptr<OffsetToTarget> CalculateOffsetToTargetCommand::get_offset(){
  return offset_to_target_;
}

CalculateOffsetTf2Command::CalculateOffsetTf2Command(
    std::shared_ptr<DroneControl> drone_control,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    std::shared_ptr<tf2_ros::TransformListener> tf_listener,
    std::shared_ptr<OffsetToTarget> offset_to_target,
    const std::string target_frame,
    const double max_age_seconds)
		: CalculateOffsetToTargetCommand(drone_control, offset_to_target),
      	target_frame_(target_frame), max_age_seconds_(max_age_seconds), tf_buffer_(tf_buffer), tf_listener_(tf_listener) {}

Command::Result CalculateOffsetTf2Command::exec(){
    geometry_msgs::msg::TransformStamped t_center_of_drone;
    geometry_msgs::msg::TransformStamped t_magnet;
    try {
        t_center_of_drone = tf_buffer_->lookupTransform(
          	"drone_center",
			target_frame_,
            tf2::TimePointZero);

        t_magnet = tf_buffer_->lookupTransform(
            "magnet_tip",
            target_frame_,
            tf2::TimePointZero);

        //Check the age of the transformation, check only one, as both are transformed to the marker
	    rclcpp::Time now = drone_control_->get_clock()->now();
	    rclcpp::Time transform_time = t_center_of_drone.header.stamp;
        double age_seconds = (now - transform_time).seconds();
        if (age_seconds > max_age_seconds_) {
            RCLCPP_WARN(drone_control_->get_logger(), "CalculateOffsetTf2Command: Transform is too old! Age: %.2fs", age_seconds);
            return Command::Result::Failure;
        }

        offset_to_target_->x_ = t_center_of_drone.transform.translation.x;
        offset_to_target_->y_ = t_center_of_drone.transform.translation.y;
        offset_to_target_->z_ = t_magnet.transform.translation.z; // z dist to magnet
        offset_to_target_->yaw_ = 0.f; // is ignored
        offset_to_target_->yaw_rate_ = 0.f; // is ignored

        return Command::Result::Complete;

    } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(drone_control_->get_logger(), "CalculateOffsetTf2Command: Could not transform magnet_tip to marker_42: %s", ex.what());
        return Command::Result::Error;
    }
}
CalculateOffsetToGlobalPoseCommand::CalculateOffsetToGlobalPoseCommand(
    std::shared_ptr<DroneControl> drone_control,
    std::shared_ptr<OffsetToTarget> offset_to_target,
    std::shared_ptr<geometry_msgs::msg::PoseStamped> global_target_pose)
    : CalculateOffsetToTargetCommand(drone_control, offset_to_target),
      global_target_pose_(global_target_pose){}

Command::Result CalculateOffsetToGlobalPoseCommand::exec() {

  	//sanity check
    if (!global_target_pose_) {
        RCLCPP_ERROR(drone_control_->get_logger(), "CalculateOffsetToGlobalPoseCommand: global_target_pose_ is null!");
        return Command::Result::Error;
    }
	auto curr_pose = drone_control_->get_current_position();
    if (!curr_pose) {
        RCLCPP_ERROR(drone_control_->get_logger(), "CalculateOffsetToGlobalPoseCommand: curr_pose is null!");
        return Command::Result::Error;
    }

    // Convert poseStamped (current) to tf2
    tf2::Transform world_to_drone;
    tf2::fromMsg(curr_pose->pose, world_to_drone);

     // Convert poseStamped (target) to tf2
    tf2::Transform world_to_target;
    tf2::fromMsg(global_target_pose_->pose, world_to_target);

    // target relative to the drone
    tf2::Transform drone_to_target = world_to_drone.inverse() * world_to_target;

    offset_to_target_->x_ = drone_to_target.getOrigin().x();
    offset_to_target_->y_ = drone_to_target.getOrigin().y();
    offset_to_target_->z_ = drone_to_target.getOrigin().z();

    offset_to_target_->yaw_ = 0.0f; // is ignored
    offset_to_target_->yaw_rate_ = 0.0f; // is ignored

    return Command::Result::Complete;
}

CreateGlobalTargetCommand::CreateGlobalTargetCommand(
    std::shared_ptr<DroneControl> drone_control,
    std::shared_ptr<OffsetToTarget> offset,
    std::shared_ptr<geometry_msgs::msg::PoseStamped> global_pose)
    : drone_control_(drone_control),
      offset_(offset),
      global_pose_(global_pose) {}

Command::Result CreateGlobalTargetCommand::exec() {
    auto current_pose = drone_control_->get_current_position();

    // Convert poseStamped (current) to tf2
    tf2::Transform drone_tf;
    tf2::fromMsg(current_pose->pose, drone_tf);

    // transform into local offset
    tf2::Transform local_offset_tf;
    local_offset_tf.setOrigin(tf2::Vector3(offset_->x_, offset_->y_, offset_->z_));

    // use yaw
    tf2::Quaternion q_offset;
    q_offset.setRPY(0.0, 0.0, offset_->yaw_);
    local_offset_tf.setRotation(q_offset);

    //global target
    tf2::Transform global_target_tf = drone_tf * local_offset_tf;

    //Convert tf2 (target) to poseStamped
    global_pose_->header.stamp = drone_control_->get_clock()->now();
    global_pose_->header.frame_id = current_pose->header.frame_id;

    tf2::toMsg(global_target_tf, global_pose_->pose);

    RCLCPP_INFO(drone_control_->get_logger(),
        "CreateGlobalTargetCommand: Created global target at [X: %.2f, Y: %.2f, Z: %.2f]",
        global_pose_->pose.position.x,
        global_pose_->pose.position.y,
        global_pose_->pose.position.z);

    return Command::Result::Complete;
}

GoRelativeToTargetCommand::GoRelativeToTargetCommand(
    std::shared_ptr<DroneControl> drone_control,
    std::shared_ptr<CalculateOffsetToTargetCommand> targeter,
    std::shared_ptr<OffsetToTarget> desired_offset_to_target,
    const double desired_offset_reached_epsilon,
    const double max_speed,
    const DesiredType x_type,
    const DesiredType y_type,
    const DesiredType z_type,
    const DesiredType yaw_type,
    const DesiredType yaw_rate_type,
    const bool use_vel_controller)
    	: drone_control_(drone_control), targeter_(targeter), desired_offset_to_target_(desired_offset_to_target),
    	desired_offset_reached_epsilon_(desired_offset_reached_epsilon), max_speed_(max_speed),
    	x_type_(x_type), y_type_(y_type), z_type_(z_type), yaw_type_(yaw_type), yaw_rate_type_(yaw_rate_type),
        use_vel_controller_(use_vel_controller){}

Command::Result GoRelativeToTargetCommand::exec(){

	// in control?
  	if(!drone_control_->in_control()) {
     	return Result::Error;
  	}

    auto targeter_result = targeter_->exec();
    if (Command::Result::Failure == targeter_result) {return Command::Result::Failure;}

    auto offset = targeter_->get_offset();

    double tx = 0;
    if((x_type_ == DesiredType::LessThanOrEqual && offset-> x_ > desired_offset_to_target_->x_)
        || (x_type_ == DesiredType::GreaterThanOrEqual && offset-> x_ < desired_offset_to_target_->x_)
        || x_type_ == DesiredType::Equal){
        tx = offset->x_ -  desired_offset_to_target_->x_;
    }

    double ty = 0;
    if((y_type_ == DesiredType::LessThanOrEqual && offset-> y_ > desired_offset_to_target_->y_)
        || (y_type_ == DesiredType::GreaterThanOrEqual && offset-> y_ < desired_offset_to_target_->y_)
        || y_type_ == DesiredType::Equal){
        ty = offset->y_ - desired_offset_to_target_->y_;
    }

    double tz = 0;
    if((z_type_ == DesiredType::LessThanOrEqual && offset-> z_ > desired_offset_to_target_->z_)
        || (z_type_ == DesiredType::GreaterThanOrEqual && offset-> z_ < desired_offset_to_target_->z_)
        || z_type_ == DesiredType::Equal){
        tz = offset->z_ - desired_offset_to_target_->z_;
    }

    double dist = std::sqrt(std::pow(tx, 2) +
                            std::pow(ty, 2) +
                            std::pow(tz, 2));

    double vx = tx;
    double vy = ty;
    double vz = tz;

    if (use_vel_controller_) {
		vx = drone_control_->get_vel_controller()->next_velocity_x(tx);
    	vy = drone_control_->get_vel_controller()->next_velocity_y(ty);
    	vz = drone_control_->get_vel_controller()->next_velocity_z(tz);
    }

    RCLCPP_INFO(drone_control_->get_logger(),
		"GoRelativeToTargetCommand: d=%f d(%f, %f, %f) t(%f, %f, %f), v(%f, %f, %f)",
		dist,
        offset->x_,offset->y_,offset->z_,
        tx, ty, tz,
        vx, vy, vz);

    if (dist <= desired_offset_reached_epsilon_) {
        return Command::Result::Complete;
    }

    VelocityCommand v_cmd(drone_control_, std::make_shared<OffsetToTarget>(vx, vy, vz, 0., 0.), max_speed_, 0.0);
    v_cmd.exec();
    return Command::Result::Running;
}

VelocityCommand::VelocityCommand(std::shared_ptr<DroneControl> drone_control,
	std::shared_ptr<OffsetToTarget> desired_speed_to_target,
    const double max_speed,
    const double seconds)
		: drone_control_(drone_control), desired_speed_to_target_(desired_speed_to_target), max_speed_(max_speed), seconds_(seconds) {}


Command::Result VelocityCommand::exec(){
    // in control?
  	if(!drone_control_->in_control()) {
     	return Result::Error;
  	}

    //init time
    if (!start_time_.has_value()){
		start_time_ = drone_control_->get_clock()->now();
	}



    OffsetToTarget offboard_vel(std::clamp(desired_speed_to_target_->x_, -max_speed_, max_speed_),
                                std::clamp(desired_speed_to_target_->y_, -max_speed_, max_speed_),
                                std::clamp(desired_speed_to_target_->z_, -max_speed_, max_speed_),
                                std::clamp(desired_speed_to_target_->yaw_, (float)-max_speed_, (float)max_speed_),
                                std::clamp(desired_speed_to_target_->yaw_rate_, (float)-max_speed_, (float)max_speed_));
	RCLCPP_INFO(drone_control_->get_logger(),
		"VelocityCommand: %s",
		offboard_vel.to_string().c_str());

    drone_control_->set_offboard_velocity(offboard_vel.x_, offboard_vel.y_, offboard_vel.z_, offboard_vel.yaw_, offboard_vel.yaw_rate_);

    //check if done
    rclcpp::Time now = drone_control_->get_clock()->now();
    double time_since_start = (now - start_time_.value()).seconds();
    if (time_since_start > seconds_) {
        start_time_.reset();
      	return Result::Complete;
    }

    return Result::Running;
}

ThrustCommand::ThrustCommand(std::shared_ptr<DroneControl> drone_control,
             const double thrust,
             const double seconds)
		: drone_control_(drone_control), thrust_(thrust), seconds_(seconds) {}


Command::Result ThrustCommand::exec(){
    // in control?
  	if(!drone_control_->in_control()) {
     	return Result::Error;
  	}

    //init time
    if (!start_time_.has_value()){
		start_time_ = drone_control_->get_clock()->now();
	}

    drone_control_->set_offboard_thrust(thrust_);

    //check if done
    rclcpp::Time now = drone_control_->get_clock()->now();
    double time_since_start = (now - start_time_.value()).seconds();
    if (time_since_start > seconds_) {
        start_time_.reset();
      	return Result::Complete;
    }

    return Result::Running;
}

OffboardCommand::OffboardCommand(std::shared_ptr<DroneControl> drone_control)
		: drone_control_(drone_control) {}


Command::Result OffboardCommand::exec(){
    // take control
    return (Command::Result)drone_control_->take_control();
}

CenterBellowTargetCommandQueue::CenterBellowTargetCommandQueue(
    std::shared_ptr<DroneControl> drone_control,
    std::shared_ptr<CalculateOffsetToTargetCommand> targeter,
    const double distance_below,
    const double max_speed,
    const bool reset_on_complete)
    	: CommandQueue(drone_control, "CenterBellowTargetCommandQueue", reset_on_complete),
    	targeter_(targeter), distance_below_(distance_below), max_speed_(max_speed){

		RCLCPP_WARN(drone_control_->get_logger(), "%s: adding commands", name_.c_str());

        //offboard
        add_command(std::make_shared<GotoCommand>(
                    	std::make_shared<OffboardCommand>(
            				drone_control),
                    "OffboardCommand",
            		1, 0, 0, 0, true, true));

		//get height
        targeter_->exec();
        double z = targeter_->get_offset()->z_;

        // center x and y
        add_command(std::make_shared<GotoCommand>(
                    std::make_shared<GoRelativeToTargetCommand>(
            			drone_control,
            			targeter_,
            			std::make_shared<OffsetToTarget>(0., 0., z, 0., 0.),
            			0.04,
            			max_speed_,
			            GoRelativeToTargetCommand::DesiredType::Equal,
            			GoRelativeToTargetCommand::DesiredType::Equal,
            			GoRelativeToTargetCommand::DesiredType::Ignore,
            			GoRelativeToTargetCommand::DesiredType::Ignore,
            			GoRelativeToTargetCommand::DesiredType::Ignore),
                    "GoRelativeToTargetCommand",
            		1, 1, 0, 0, true, true));

        // fly to desired height
        add_command(std::make_shared<GotoCommand>(
                    std::make_shared<GoRelativeToTargetCommand>(
            			drone_control,
            			targeter_,
            			std::make_shared<OffsetToTarget>(0., 0., distance_below_, 0., 0.),
            			0.04,
            			max_speed_,
			            GoRelativeToTargetCommand::DesiredType::Equal,
            			GoRelativeToTargetCommand::DesiredType::Equal,
            			GoRelativeToTargetCommand::DesiredType::Equal,
            			GoRelativeToTargetCommand::DesiredType::Ignore,
            			GoRelativeToTargetCommand::DesiredType::Ignore),
                    "GoRelativeToTargetCommand",
            		1, 1, 0, 0, true, true));

        RCLCPP_WARN(drone_control_->get_logger(), "%s: added commands", name_.c_str());
}

MagnetDockAtTargetCommandQueue::MagnetDockAtTargetCommandQueue(
	std::shared_ptr<DroneControl> drone_control,
	std::shared_ptr<MagnetServiceClient> magnet_client,
	std::shared_ptr<CalculateOffsetToTargetCommand> targeter,
	const double max_speed,
	const bool reset_on_complete)
    	: CommandQueue(drone_control, "MagnetDockAtTargetCommandQueue", reset_on_complete),
    	magnet_client_(magnet_client), targeter_(targeter), max_speed_(max_speed){

		RCLCPP_WARN(drone_control_->get_logger(), "%s: adding commands", name_.c_str());

        //offboard
        add_command(std::make_shared<GotoCommand>(
                    	std::make_shared<OffboardCommand>(
            				drone_control),
                    "OffboardCommand",
            		1, 0, 0, 0, true, true));

        // start to ready magnet
        add_command(std::make_shared<GotoCommand>(
                    	std::make_shared<MagnetCommand>(
                    	drone_control_,
                    	magnet_client_,
                    	MagnetServiceClient::Direction::On),
                    "MagnetCommand",
                    1, 0, 1, 0, true, true
        ));

        // center below
        add_command(std::make_shared<GotoCommand>(
            			std::make_shared<CenterBellowTargetCommandQueue>(
                    		drone_control_,
                    		targeter_,
                    		0.30,
                    		max_speed_,
                   	 		true),
            			"CenterBellowTargetCommandQueue"
        ));

        // wait till magnet is ready
        add_command(std::make_shared<GotoCommand>(
            			std::make_shared<MagnetCommand>(
                    	drone_control_,
                    	magnet_client_,
                    	MagnetServiceClient::Direction::On),
            			"MagnetCommand",
                    	1, 0, -1, 0, true, true
        ));

        // center below
        add_command(std::make_shared<GotoCommand>(
            			std::make_shared<CenterBellowTargetCommandQueue>(
                    		drone_control_,
                    		targeter_,
                    		0.10,
                    		max_speed_,
                   	 		true),
            			"CenterBellowTargetCommandQueue"
        ));

        // try to touch target
        add_command(std::make_shared<GotoCommand>(
            			std::make_shared<CenterBellowTargetCommandQueue>(
                    		drone_control_,
                    		targeter_,
                    		0.02,
                    		max_speed_,
                   	 		true),
            			"CenterBellowTargetCommandQueue",
                    	1, 1, 0, 0, true, false
        ));

        // blind flight to the marker
        add_command(std::make_shared<GotoCommand>(
            			std::make_shared<VelocityCommand>(
                    	drone_control_,
                    	targeter_->get_offset(),
                    	max_speed_,
                    	3.0),
            		"VelocityCommand"
        ));

		//create target below drone
		auto global_target = std::make_shared<geometry_msgs::msg::PoseStamped>();
		add_command(std::make_shared<GotoCommand>(
				std::make_shared<CreateGlobalTargetCommand>(
					  this->drone_control_,
						std::make_shared<OffsetToTarget>(0.0, 0.0, -0.4, 0.0, 0.0),
						global_target
				),
				"CreateGlobalTargetCommand")
		);

		//thrust up (not so much to stay level -> slower decent than just falling
		add_command(std::make_shared<GotoCommand>(
			std::make_shared<ThrustCommand>(
				drone_control_,
				0.3,
				3.0),
			"ThrustCommand",
			3, 0, 1, 0, true, true));

		//wait till flying down
		add_command(std::make_shared<GotoCommand>(
			std::make_shared<IsMovingCommand>(
				drone_control_,
				IsMovingCommand::Direction::Down,
				0.05),
			"IsMovingCommand",
			1, -1, -1, 0, true, false));

		//go to position bellow target, if docking failed
		add_command(std::make_shared<GotoCommand>(
			std::make_shared<GoRelativeToTargetCommand>(
				drone_control_,
				std::make_shared<CalculateOffsetToGlobalPoseCommand>(
					drone_control_,
					std::make_shared<OffsetToTarget>(0, 0, 0, 0, 0),
					global_target // flyies here
				),
				std::make_shared<OffsetToTarget>(0., 0., 0., 0., 0.),
				0.04,
				max_speed_,
				GoRelativeToTargetCommand::DesiredType::Equal,
				GoRelativeToTargetCommand::DesiredType::Equal,
				GoRelativeToTargetCommand::DesiredType::Equal,
				GoRelativeToTargetCommand::DesiredType::Ignore,
				GoRelativeToTargetCommand::DesiredType::Ignore),
			"GoRelativeToTargetCommand",
			3, 1, 0, 0, true, true
		));

        //publish up to put less force on the propellers
        add_command(std::make_shared<GotoCommand>(
            std::make_shared<ThrustCommand>(
                drone_control_,
                0.1,
                2.0),
            "ThrustCommand",
            1, 0, 0, 0, true, true));

        // disarm
        add_command(std::make_shared<GotoCommand>(
                    	std::make_shared<ArmCommand>(
                    	drone_control_,
                    	false),
                        "ArmCommand"));

        RCLCPP_WARN(drone_control_->get_logger(), "%s: added commands", name_.c_str());
}

MagnetDetachCommandQueue::MagnetDetachCommandQueue(
	std::shared_ptr<DroneControl> drone_control,
	std::shared_ptr<MagnetServiceClient> magnet_client,
	const double max_speed,
	const bool reset_on_complete)
    	: CommandQueue(drone_control, "MagnetDetachCommandQueue", reset_on_complete),
    	magnet_client_(magnet_client), max_speed_(max_speed){

		RCLCPP_WARN(drone_control_->get_logger(), "%s: adding commands", name_.c_str());

    	//offboard
        add_command(std::make_shared<GotoCommand>(
                    	std::make_shared<OffboardCommand>(
            				drone_control),
                    "OffboardCommand",
            		1, 0, 0, 0, true, true));

        //publish up velocity to get arming allowance
		add_command(std::make_shared<GotoCommand>(
                    std::make_shared<VelocityCommand>(
                    	drone_control_,
                    	std::make_shared<OffsetToTarget>(0, 0, max_speed_, 0, 0),
                    	max_speed_,
                    	1.0),
                    "VelocityCommand",
                    1, 0, 0, 0, true, true));

        //arm
        add_command(std::make_shared<GotoCommand>(
                    std::make_shared<ArmCommand>(
            			drone_control_,
            			true),
            		"ArmCommand",
            		1, -1, -1, -1, false, false));

        //publish till in air
        add_command(std::make_shared<GotoCommand>(
                    std::make_shared<ThrustCommand>(
                    	drone_control_,
                    	0.3,
                    	1.0),
                    "ThrustCommand",
                    1, 0, 1, 0, true, true));

        //go back to publishing when not in air
        add_command(std::make_shared<GotoCommand>(
            std::make_shared<IsInAirCommand>(
            drone_control_),
            "IsInAirCommand",
            1, -1, 0, 0, true, false));

		//create target below drone
		auto global_target = std::make_shared<geometry_msgs::msg::PoseStamped>();
		add_command(std::make_shared<GotoCommand>(
                std::make_shared<CreateGlobalTargetCommand>(
                      this->drone_control_,
    					std::make_shared<OffsetToTarget>(0.0, 0.0, -0.4, 0.0, 0.0),
    					global_target
                ),
                "CreateGlobalTargetCommand")
		);

        //make magnet go down
        add_command(std::make_shared<GotoCommand>(
            std::make_shared<MagnetCommand>(
            drone_control_,
            magnet_client_,
            MagnetServiceClient::Direction::Off),
            "MagnetCommand",
            1, 0, 1, 0, true, true));

		//thrust up (not so much to stay level -> slower decent than just falling
		add_command(std::make_shared<GotoCommand>(
            std::make_shared<ThrustCommand>(
                drone_control_,
                0.3,
                5.0),
            "ThrustCommand",
            1, 0, 1, 0, true, true));

		//wait till flying down
		add_command(std::make_shared<GotoCommand>(
			std::make_shared<IsMovingCommand>(
				drone_control_,
				IsMovingCommand::Direction::Down,
				0.05),
			"IsMovingCommand",
			1, -1, -1, 0, true, false));


		add_command(std::make_shared<GotoCommand>(
			std::make_shared<GoRelativeToTargetCommand>(
				drone_control_,
				std::make_shared<CalculateOffsetToGlobalPoseCommand>(
					drone_control_,
					std::make_shared<OffsetToTarget>(0, 0, 0, 0, 0),
					global_target // flyies here
				),
				std::make_shared<OffsetToTarget>(0., 0., 0., 0., 0.),
				0.04,
				max_speed_,
				GoRelativeToTargetCommand::DesiredType::Equal,
				GoRelativeToTargetCommand::DesiredType::Equal,
				GoRelativeToTargetCommand::DesiredType::Equal,
				GoRelativeToTargetCommand::DesiredType::Ignore,
				GoRelativeToTargetCommand::DesiredType::Ignore),
			"GoRelativeToTargetCommand",
			1, 1, 0, 0, true, true
		));


        RCLCPP_WARN(drone_control_->get_logger(), "%s: added commands", name_.c_str());
}


IsMovingCommand::IsMovingCommand(
	std::shared_ptr<DroneControl> drone_control,
    const Direction direction,
	const double moving_threshold)
		: drone_control_(drone_control), direction_(direction), moving_threshold_(moving_threshold){}

Command::Result IsMovingCommand::exec(){

	auto current_twist = drone_control_->get_current_twist();

	if (!current_twist) {
    	RCLCPP_INFO(drone_control_->get_logger(), "Waiting for twist information");
    	return Result::Running;
	}

	bool is_moving = false;
  	switch(direction_){
		case Direction::Up:
			is_moving = current_twist->twist.linear.z > moving_threshold_;
            break;
        case Direction::Down:
			is_moving = current_twist->twist.linear.z < -moving_threshold_;
            break;
        case Direction::Left:
			is_moving = current_twist->twist.linear.y > moving_threshold_;
            break;
        case Direction::Right:
			is_moving = current_twist->twist.linear.y < -moving_threshold_;
            break;
        case Direction::Forward:
			is_moving = current_twist->twist.linear.x > moving_threshold_;
            break;
        case Direction::Backward:
			is_moving = current_twist->twist.linear.x < -moving_threshold_;
            break;
  	}

	return is_moving ? Command::Result::Complete : Command::Result::Failure;
}