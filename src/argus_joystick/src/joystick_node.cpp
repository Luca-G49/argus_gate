#include <memory>
#include <chrono>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/joy.hpp"
#include <SFML/Window/Joystick.hpp>

using namespace std::chrono_literals;

/**
 * @class JoystickNode
 * @brief Reads hardware joystick input via SFML and publishes generic sensor_msgs/Joy messages.
 */
class JoystickNode : public rclcpp::Node {
public:
    JoystickNode() : Node("joystick_node") {
        // --- 1. ROS2 PUBLISHER SETUP ---
        publisher_ = this->create_publisher<sensor_msgs::msg::Joy>("joy", 10);

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
        }

        // --- 3. TIMER FOR PERIODIC PUBLISHING (50Hz) ---
        timer_ = this->create_wall_timer(20ms, std::bind(&JoystickNode::read_and_publish, this));

        RCLCPP_INFO(this->get_logger(), "--- Joystick Node Active ---");
    }

private:
    /**
     * @brief Reads all joystick axes and buttons and publishes a Joy message.
     */
    void read_and_publish() {
        // Refresh joystick state
        sf::Joystick::update();

        sensor_msgs::msg::Joy msg;
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = "joystick_frame";

        // --- 1. INITIALIZE DEFAULT COMMAND VALUES ---
        // Pre-allocate vectors for axes (8) and buttons (14)
        msg.axes.assign(8, 0.0f);
        msg.buttons.assign(14, 0);

        if (joystick_id_ != -1 && sf::Joystick::isConnected(joystick_id_)) {
            // --- 2. READ ANALOG AXES (Normalized to [-1.0, 1.0]) ---
            // Mapping follows standard PS4/Xbox patterns via SFML
            msg.axes[0] = sf::Joystick::getAxisPosition(joystick_id_, sf::Joystick::X) / 100.0f;  // Left Stick L/R
            msg.axes[1] = sf::Joystick::getAxisPosition(joystick_id_, sf::Joystick::Y) / -100.0f; // Left Stick U/D (Inverted)
            msg.axes[2] = sf::Joystick::getAxisPosition(joystick_id_, sf::Joystick::Z) / 100.0f;  // L2 Trigger
            msg.axes[3] = sf::Joystick::getAxisPosition(joystick_id_, sf::Joystick::R) / -100.0f; // Right Stick U/D (Inverted)
            msg.axes[4] = sf::Joystick::getAxisPosition(joystick_id_, sf::Joystick::U) / 100.0f;  // Right Stick L/R
            msg.axes[5] = sf::Joystick::getAxisPosition(joystick_id_, sf::Joystick::V) / 100.0f;  // R2 Trigger
            msg.axes[6] = sf::Joystick::getAxisPosition(joystick_id_, sf::Joystick::PovX) / 100.0f; // D-Pad H
            msg.axes[7] = sf::Joystick::getAxisPosition(joystick_id_, sf::Joystick::PovY) / 100.0f; // D-Pad V

            // --- 3. READ DIGITAL BUTTONS ---
            for (unsigned int i = 0; i < msg.buttons.size(); ++i) {
                msg.buttons[i] = sf::Joystick::isButtonPressed(joystick_id_, i) ? 1 : 0;
            }
        } else {
            // Data remains zeroed if joystick is lost to ensure safety
        }

        // --- 4. PUBLISH MESSAGE ---
        publisher_->publish(msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr publisher_;
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
