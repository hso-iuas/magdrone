#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "cpp_drone_control/drone_control_mavros.hpp"
#include "mavros_commander.hpp"
#include "cpp_service_client/magnet_service_client.hpp"

using namespace std::chrono_literals;

/**
 * @class MavrosCommanderNode
 * @brief ROS2 node running the flight logic.
 */
class MavrosCommanderNode : public rclcpp::Node {
    public:
        MavrosCommanderNode() : Node("mavros_commander_node") {
            RCLCPP_INFO(this->get_logger(), "Created MavrosCommanderNode");

            auto magnet_client = std::make_shared<MagnetServiceClient>(this);
            auto drone_control = std::make_shared<MavrosDroneControl>(this);

            mavros_commander_ = std::make_shared<MavrosCommander>(drone_control, magnet_client, this);
        }

    private:
        std::shared_ptr<MavrosCommander> mavros_commander_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MavrosCommanderNode>());
    rclcpp::shutdown();
}
