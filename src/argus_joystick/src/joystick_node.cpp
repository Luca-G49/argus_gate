#include <memory>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include "argus_msgs/msg/plc_command.hpp"
#include <SFML/Window/Joystick.hpp>

using namespace std::chrono_literals;

/**
 * @class JoystickNode
 * @brief Reads PS4 joystick input via SFML and publishes corresponding PlcCommand messages.
 */
class JoystickNode : public rclcpp::Node {
public:
    JoystickNode() : Node("joystick_node") {
        // --- 1. ROS2 PUBLISHER SETUP ---
        publisher_ = this->create_publisher<argus_msgs::msg::PlcCommand>("plc_command", 10);

        // --- 2. JOYSTICK INITIALIZATION ---
        joystick_id_ = -1;
        sf::Joystick::update(); // Refresh joystick state
        
        for (unsigned int i = 0; i < sf::Joystick::Count; ++i) {
            if (sf::Joystick::isConnected(i)) {
                joystick_id_ = i;
                RCLCPP_INFO(this->get_logger(), "Joystick connected: ID %d", i);
                break;
            }
        }

        if (joystick_id_ == -1) {
            RCLCPP_ERROR(this->get_logger(), "No joystick detected!");
            // We don't throw to allow the node to stay alive for hot-plugging if needed
        }

        // --- 3. TIMER FOR PERIODIC PUBLISHING (50Hz) ---
        timer_ = this->create_wall_timer(20ms, std::bind(&JoystickNode::publish_command, this));

        RCLCPP_INFO(this->get_logger(), "--- Joystick Node Active ---");
    }

private:
    /**
     * @brief Reads joystick axes and publishes PlcCommand message.
     */
    void publish_command() {
        // MUST call update() to refresh hardware state
        sf::Joystick::update();

        argus_msgs::msg::PlcCommand msg;

        // --- 1. INITIALIZE DEFAULT COMMAND VALUES ---
        msg.ack  = false;
        msg.exec = false;
        msg.fire = false;
        msg.mode = 0;
        msg.pitch_override = 100;
        msg.yaw_override   = 100;
        msg.target_pitch   = 0;
        msg.target_yaw     = 0;

        if (joystick_id_ != -1 && sf::Joystick::isConnected(joystick_id_)) {
            // --- 2. READ RIGHT ANALOG STICK (PS4: U=Axis 4, R=Axis 5) ---
            float axis_x = sf::Joystick::getAxisPosition(joystick_id_, sf::Joystick::U);
            float axis_y = sf::Joystick::getAxisPosition(joystick_id_, sf::Joystick::R);

            // --- 3. APPLY DEADZONE AND MAP TO BOOLEAN JOG COMMANDS ---
            const float deadzone = 15.0f;
            msg.pitch_jog_p = axis_y < -deadzone; // stick up
            msg.pitch_jog_n = axis_y > deadzone;  // stick down
            msg.yaw_jog_p   = axis_x > deadzone;  // stick right
            msg.yaw_jog_n   = axis_x < -deadzone; // stick left
        } else {
            // Reset jog commands if joystick is lost
            msg.pitch_jog_p = msg.pitch_jog_n = msg.yaw_jog_p = msg.yaw_jog_n = false;
        }

        // --- 4. PUBLISH MESSAGE ---
        publisher_->publish(msg);
    }

    rclcpp::Publisher<argus_msgs::msg::PlcCommand>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int joystick_id_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JoystickNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
