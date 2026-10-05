#include "cpp_drone_control/drone_control_mavros.hpp"


using namespace std::chrono_literals;

MavrosDroneControl::MavrosDroneControl(rclcpp::Node* node) : node_(node) {
    RCLCPP_INFO(get_logger(), "Creating Mavros Drone Control class");

    init_topics();
    init_services();

    std::ifstream is("config.ini");

    // Check if the file opened successfully
    if (!is.is_open()) {
        throw std::runtime_error("Could not open config.ini");
    }
	  RCLCPP_INFO(this->get_logger(), "ini fs okay");

    // Load the INI file using the constructor
    ini::IniFile ini_file(is);

    RCLCPP_INFO(this->get_logger(), "ini loaded");

    //create vel controller from ini
    auto vel_controller = std::make_shared<XyzVelocityController>(
        std::make_shared<PidVelocityController>(
            ini_file["PID"]["x_kp"].as<double>(),
            ini_file["PID"]["x_ki"].as<double>(),
            ini_file["PID"]["x_kd"].as<double>(),
            ini_file["PID"]["x_ta"].as<double>(),
            ini_file["PID"]["x_a"].as<double>(),
            ini_file["PID"]["x_i_vec_length"].as<int>()),
        
        std::make_shared<PidVelocityController>(
            ini_file["PID"]["y_kp"].as<double>(),
            ini_file["PID"]["y_ki"].as<double>(),
            ini_file["PID"]["y_kd"].as<double>(),
            ini_file["PID"]["y_ta"].as<double>(),
            ini_file["PID"]["y_a"].as<double>(),
            ini_file["PID"]["y_i_vec_length"].as<int>()),
        
        std::make_shared<PidVelocityController>(
            ini_file["PID"]["z_kp"].as<double>(),
            ini_file["PID"]["z_ki"].as<double>(),
            ini_file["PID"]["z_kd"].as<double>(),
            ini_file["PID"]["z_ta"].as<double>(),
            ini_file["PID"]["z_a"].as<double>(),
            ini_file["PID"]["z_i_vec_length"].as<int>())
    );
    set_vel_controller(vel_controller);

    RCLCPP_INFO(get_logger(), "Creation of Mavros Drone Control complete");
}

void MavrosDroneControl::init_topics() {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);

    state_sub_ = node_->create_subscription<mavros_msgs::msg::State>(
        "mavros/state", 10,
        [this](mavros_msgs::msg::State::SharedPtr msg) { mavros_state_ = msg; });

    pos_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
        "mavros/setpoint_position/local", 10);

    pos_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/mavros/local_position/pose", rclcpp::QoS(10).best_effort(),
        [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        current_position_ = msg;
    });

    ext_state_sub_ = node_->create_subscription<mavros_msgs::msg::ExtendedState>(
        "mavros/extended_state", rclcpp::QoS(10).best_effort(),
        [this](mavros_msgs::msg::ExtendedState::SharedPtr msg) {
        ext_state_ = msg;
    });

    twist_sub_ = node_->create_subscription<geometry_msgs::msg::TwistStamped>(
        "/mavros/local_position/velocity_local", rclcpp::SensorDataQoS(),
        [this](geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        current_twist_ = msg;
    });

    attitude_pub_ = node_->create_publisher<mavros_msgs::msg::AttitudeTarget>("/mavros/setpoint_raw/attitude", 10);
    velocity_pub_ = node_->create_publisher<mavros_msgs::msg::PositionTarget>("/mavros/setpoint_raw/local", 10);
}

DroneControl::Result MavrosDroneControl::set_vel_controller(std::shared_ptr<XyzVelocityController> vel_ctrl){
	vel_controller_ = vel_ctrl;
    auto vel_controller = get_vel_controller();

    auto ctrl_x_base = vel_controller->get_ctrl_x();
    auto ctrl_y_base = vel_controller->get_ctrl_y();
    auto ctrl_z_base = vel_controller->get_ctrl_z();

    auto ctrl_x_pid = std::dynamic_pointer_cast<PidVelocityController>(ctrl_x_base);
    auto ctrl_y_pid = std::dynamic_pointer_cast<PidVelocityController>(ctrl_y_base);
    auto ctrl_z_pid = std::dynamic_pointer_cast<PidVelocityController>(ctrl_z_base);

    if (!(ctrl_x_pid && ctrl_y_pid && ctrl_z_pid)) {
        return DroneControl::Result::Error;
    }

    // safe to file
    ini::IniFile ini_file;
    // X-Axis
    ini_file["PID"]["x_kp"] = ctrl_x_pid->kp_;
    ini_file["PID"]["x_ki"] = ctrl_x_pid->ki_;
    ini_file["PID"]["x_kd"] = ctrl_x_pid->kd_;
    ini_file["PID"]["x_ta"] = ctrl_x_pid->ta_;
    ini_file["PID"]["x_a"]  = ctrl_x_pid->a_;
    ini_file["PID"]["x_i_vec_length"] = ctrl_x_pid->i_vec_length_;

    // Y-Axis
    ini_file["PID"]["y_kp"] = ctrl_y_pid->kp_;
    ini_file["PID"]["y_ki"] = ctrl_y_pid->ki_;
    ini_file["PID"]["y_kd"] = ctrl_y_pid->kd_;
    ini_file["PID"]["y_ta"] = ctrl_y_pid->ta_;
    ini_file["PID"]["y_a"]  = ctrl_y_pid->a_;
    ini_file["PID"]["y_i_vec_length"] = ctrl_y_pid->i_vec_length_;

    // Z-Axis
    ini_file["PID"]["z_kp"] = ctrl_z_pid->kp_;
    ini_file["PID"]["z_ki"] = ctrl_z_pid->ki_;
    ini_file["PID"]["z_kd"] = ctrl_z_pid->kd_;
    ini_file["PID"]["z_ta"] = ctrl_z_pid->ta_;
    ini_file["PID"]["z_a"]  = ctrl_z_pid->a_;
    ini_file["PID"]["z_i_vec_length"] = ctrl_z_pid->i_vec_length_;

    std::ofstream os("config.ini");
    if (!os.is_open()) {
        return DroneControl::Result::Error;
    }
    ini_file.encode(os);

	return DroneControl::Result::Success;
}
void MavrosDroneControl::init_services() {
    RCLCPP_INFO(get_logger(), "Creating clients");

    force_arming_client_ = std::make_shared<ForceArmingServiceClient>(node_);
    arming_client_ = std::make_shared<ArmingServiceClient>(node_);
    set_mode_client_ = std::make_shared<SetModeServiceClient>(node_);
}

geometry_msgs::msg::TwistStamped::SharedPtr MavrosDroneControl::get_current_twist(){
    return current_twist_;
}

std::shared_ptr<XyzVelocityController> MavrosDroneControl::get_vel_controller(){
    return vel_controller_;
}

bool MavrosDroneControl::is_in_air(){
    if(ext_state_ == nullptr){return false;}

    return ext_state_->landed_state == mavros_msgs::msg::ExtendedState::LANDED_STATE_IN_AIR;
}

void MavrosDroneControl::set_offboard_velocity(const double vx, const double vy, const double vz, const float yaw, const float yaw_rate, const bool ignore_yaw) {
    auto msg = mavros_msgs::msg::PositionTarget();

    msg.header.stamp = node_->now();
    msg.header.frame_id = current_position_->header.frame_id;

    //msg.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
    msg.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_BODY_NED;


    msg.type_mask = mavros_msgs::msg::PositionTarget::IGNORE_PX |
                    mavros_msgs::msg::PositionTarget::IGNORE_PY |
                    mavros_msgs::msg::PositionTarget::IGNORE_PZ |
                    //mavros_msgs::msg::PositionTarget::IGNORE_VX |
                    //mavros_msgs::msg::PositionTarget::IGNORE_VY |
                    //mavros_msgs::msg::PositionTarget::IGNORE_VZ |
                    mavros_msgs::msg::PositionTarget::IGNORE_AFX |
                    mavros_msgs::msg::PositionTarget::IGNORE_AFY |
                    mavros_msgs::msg::PositionTarget::IGNORE_AFZ
                    //mavros_msgs::msg::PositionTarget::FORCE |
    ;

    if (ignore_yaw){
        msg.type_mask = msg.type_mask |
                        mavros_msgs::msg::PositionTarget::IGNORE_YAW |
                       	mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE;
    }

    msg.velocity.x = vx;
    msg.velocity.y = vy;
    msg.velocity.z = vz;

    msg.position.x = 0;
    msg.position.y = 0;
    msg.position.z = 0;

    msg.acceleration_or_force.x = 0;
    msg.acceleration_or_force.y = 0;
    msg.acceleration_or_force.z = 0;

	msg.yaw = yaw;
    msg.yaw_rate = yaw_rate;

    velocity_pub_->publish(msg);
}

void MavrosDroneControl::set_offboard_thrust(const float thrust) {
    auto msg = mavros_msgs::msg::AttitudeTarget();

    msg.header.stamp = node_->now();
    msg.header.frame_id = current_position_->header.frame_id;

    // Ignore body rates
    msg.type_mask = mavros_msgs::msg::AttitudeTarget::IGNORE_ROLL_RATE |
                    mavros_msgs::msg::AttitudeTarget::IGNORE_PITCH_RATE |
                    mavros_msgs::msg::AttitudeTarget::IGNORE_YAW_RATE;

    // perfectly level flight, maintain current yaw
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, tf2::getYaw(current_position_->pose.orientation));

    msg.orientation.x = q.x();
    msg.orientation.y = q.y();
    msg.orientation.z = q.z();
    msg.orientation.w = q.w();

    msg.thrust = thrust;

    attitude_pub_->publish(msg);
}

rclcpp::Logger MavrosDroneControl::get_logger() {
    return node_->get_logger();
}
rclcpp::Clock::SharedPtr MavrosDroneControl::get_clock() {
    return node_->get_clock();
}

DroneControl::Result MavrosDroneControl::take_control(){
    if (!in_control()) {
        for (int i = 0; i < 20; i++) {
            set_offboard_velocity(0, 0, 0, 0, 0);
        }
    if(Result::Error == set_mode_client_->make_request("OFFBOARD")){return Result::Error;}
        return Result::Running;
    }else{
        return Result::Success;
    }
}

bool MavrosDroneControl::in_control(){
	return get_mode() == "OFFBOARD";
}

geometry_msgs::msg::PoseStamped::SharedPtr MavrosDroneControl::get_current_position() {
    return current_position_;
}

DroneControl::Result MavrosDroneControl::force_arm(){
    return force_arming_client_->make_request(true);
}

DroneControl::Result MavrosDroneControl::force_disarm(){
    return force_arming_client_->make_request(false);
}

void MavrosDroneControl::set_target_position(
    const geometry_msgs::msg::PoseStamped& target) {
        pos_pub_->publish(target);
}

std::string MavrosDroneControl::get_mode() {
    if (mavros_state_)
        return mavros_state_->mode;
    else
        return "";
}