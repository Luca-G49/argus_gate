/*
 * Argus Gate - argus_teleop_joy/src/teleop_joy_node.cpp
 * Copyright (c) 2026, Luca-G49
 * All rights reserved. Licensed under MIT License.
 */

#include <memory>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/joy.hpp"
#include "argus_interfaces/msg/teleop_command.hpp"

using namespace std::chrono_literals;

/**
 * @class ArgusTeleopJoyNode
 * @brief Translator: Maps raw joystick axes/buttons to generic teleop commands.
 */
class ArgusTeleopJoyNode : public rclcpp::Node {
public:
    ArgusTeleopJoyNode() : Node("argus_teleop_joy_node") {
        // --- 1. COMMS ---
        joy_sub_    = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&ArgusTeleopJoyNode::on_joy_received, this, std::placeholders::_1));
        
        teleop_pub_ = this->create_publisher<argus_interfaces::msg::TeleopCommand>("teleop_cmd", 10);

        RCLCPP_INFO(this->get_logger(), "--- Argus Teleop Joy Active ---");
    }

private:
    /**
     * @brief Translates PS4 mapping to generic Argus Teleop intentions.
     */
    void on_joy_received(const sensor_msgs::msg::Joy::SharedPtr msg) {
        if (msg->axes.size() < 8 || msg->buttons.size() < 4) return; 

        // --- 1. MANUAL JOG LOGIC (Speeds -1.0 to 1.0) ---
        cmd.pitch_speed = msg->axes[5]; // RY Stick
        cmd.yaw_speed   = msg->axes[4]; // RX Stick

        // --- 2. MODE ARBITRATION REQUESTS ---
        if (msg->buttons[1]) cmd.requested_mode = 0; // Circle -> Request IDLE
        if (msg->buttons[2]) cmd.requested_mode = 2; // Square -> Request JOG

        // --- 4. ACTION COMMANDS ---
        cmd.execute_action = msg->buttons[0]; // Cross -> EXEC
        cmd.reset_error    = msg->buttons[3]; // Triangle -> ACK/RESET

        teleop_pub_->publish(cmd);
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<argus_interfaces::msg::TeleopCommand>::SharedPtr teleop_pub_;
    argus_interfaces::msg::TeleopCommand cmd;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArgusTeleopJoyNode>());
    rclcpp::shutdown();
    return 0;
}
