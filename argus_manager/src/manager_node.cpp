#include <memory>
#include <chrono>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/joy.hpp"
#include "argus_msgs/msg/plc_command.hpp"
#include "argus_msgs/msg/plc_status.hpp"

using namespace std::chrono_literals;

/**
 * @class ArgusManagerNode
 * @brief Industrial-grade supervisor for state arbitration and hardware safety.
 */
class ArgusManagerNode : public rclcpp::Node {
public:
    // PLC Hardware States (Reference)
    enum class PlcState : int32_t {
        STARTUP = 0, STOP = 1, READY = 2, SYNCH = 3, JOG = 4, FOLLOW = 5, ERROR = 99
    };

    ArgusManagerNode() : Node("argus_manager_node") {
        // --- 1. PARAMETERS & CONFIG ---
        this->declare_parameter("watchdog_timeout_ms", 500);
        this->declare_parameter("loop_frequency", 50.0);
        
        timeout_threshold_ = std::chrono::milliseconds(this->get_parameter("watchdog_timeout_ms").as_int());
        double freq = this->get_parameter("loop_frequency").as_double();

        // --- 2. ROS2 INTERFACES ---
        joy_sub_     = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&ArgusManagerNode::on_joy_received, this, std::placeholders::_1));
        status_sub_  = this->create_subscription<argus_msgs::msg::PlcStatus>("plc_status", 10, std::bind(&ArgusManagerNode::on_status_received, this, std::placeholders::_1));
        command_pub_ = this->create_publisher<argus_msgs::msg::PlcCommand>("plc_command", 10);

        // --- 3. CORE EXECUTION TIMER ---
        auto interval = std::chrono::duration<double>(1.0 / freq);
        timer_ = this->create_wall_timer(interval, std::bind(&ArgusManagerNode::supervisor_cycle, this));

        RCLCPP_INFO(this->get_logger(), "--- Argus Supervisor Initialized @ %.1f Hz ---", freq);
    }

private:
    /**
     * @brief Main control logic executed at fixed frequency.
     */
    void supervisor_cycle() {
        argus_msgs::msg::PlcCommand outbound_cmd;
        reset_command(outbound_cmd);

        // --- STEP 1: SYSTEM HEALTH MONITORING (Quality of Service) ---
        bool joy_alive = check_watchdog(last_joy_time_);
        bool plc_alive = check_watchdog(last_status_time_);

        if (!plc_alive) {
            handle_critical_fault("PLC Bridge Offline - Inhibiting all commands");
            return; // Absolute safety: bridge is essential for any communication
        }

        // --- STEP 2: STATE ARBITRATION ---
        PlcState current_hw_state = static_cast<PlcState>(last_status_.status);

        if (current_hw_state == PlcState::ERROR) {
            process_error_recovery(outbound_cmd);
        } 
        else if (!joy_alive) {
            handle_signal_loss("Joystick Signal Lost - Forcing Safe Stop");
            // No return here: we still want to send a STOP command to the PLC
        } 
        else {
            // --- STEP 3: LOGIC EXECUTION ---
            process_operational_logic(current_hw_state, outbound_cmd);
        }

        command_pub_->publish(outbound_cmd);
    }

    /**
     * @brief Maps operational inputs based on the current hardware state.
     */
    void process_operational_logic(PlcState state, argus_msgs::msg::PlcCommand &cmd) {
        switch (state) {
            case PlcState::READY:
            case PlcState::JOG:
                map_joystick_to_jog(cmd);
                break;
            
            case PlcState::FOLLOW:
                // Placeholder for autonomous tracking commands (e.g. from Vision Node)
                cmd.mode = 5; 
                break;

            default:
                // Keep safe defaults for STARTUP, STOP, etc.
                break;
        }
    }

    /**
     * @brief High-level mapping of Joy axes/buttons to PLC Jogging.
     */
    void map_joystick_to_jog(argus_msgs::msg::PlcCommand &cmd) {
        if (last_joy_.axes.size() < 8) return;

        const float deadzone = 0.15f;
        
        // Motion: Composite mapping (Stick + D-Pad)
        cmd.pitch_jog_p = (last_joy_.axes[1] > deadzone  || last_joy_.axes[7] > 0.5f);
        cmd.pitch_jog_n = (last_joy_.axes[1] < -deadzone || last_joy_.axes[7] < -0.5f);
        cmd.yaw_jog_p   = (last_joy_.axes[0] > deadzone  || last_joy_.axes[6] > 0.5f);
        cmd.yaw_jog_n   = (last_joy_.axes[0] < -deadzone || last_joy_.axes[6] < -0.5f);

        // Overrides & Actions
        cmd.pitch_override = 100;
        cmd.yaw_override   = 100;
        
        if (last_joy_.buttons.size() >= 2) {
            cmd.ack  = last_joy_.buttons[0]; // Cross
            cmd.exec = last_joy_.buttons[1]; // Circle
        }
    }

    void process_error_recovery(argus_msgs::msg::PlcCommand &cmd) {
        // In ERROR state, strictly allow only the ACK button
        if (!last_joy_.buttons.empty()) {
            cmd.ack = last_joy_.buttons[0];
        }
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "System in ERROR state - Waiting for ACK");
    }

    // --- UTILS & SAFETY ---
    
    bool check_watchdog(const rclcpp::Time &last_time) {
        return (this->get_clock()->now() - last_time) < timeout_threshold_;
    }

    void reset_command(argus_msgs::msg::PlcCommand &cmd) {
        cmd.pitch_jog_p = cmd.pitch_jog_n = cmd.yaw_jog_p = cmd.yaw_jog_n = false;
        cmd.ack = cmd.exec = cmd.fire = false;
        cmd.pitch_override = cmd.yaw_override = 0;
        cmd.mode = 0;
    }

    void handle_critical_fault(const std::string &msg) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "%s", msg.c_str());
    }

    void handle_signal_loss(const std::string &msg) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "%s", msg.c_str());
    }

    // Callbacks
    void on_joy_received(const sensor_msgs::msg::Joy::SharedPtr msg) {
        last_joy_ = *msg;
        last_joy_time_ = this->get_clock()->now();
    }

    void on_status_received(const argus_msgs::msg::PlcStatus::SharedPtr msg) {
        last_status_ = *msg;
        last_status_time_ = this->get_clock()->now();
    }

    // Members
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Subscription<argus_msgs::msg::PlcStatus>::SharedPtr status_sub_;
    rclcpp::Publisher<argus_msgs::msg::PlcCommand>::SharedPtr command_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    sensor_msgs::msg::Joy last_joy_;
    argus_msgs::msg::PlcStatus last_status_;
    rclcpp::Time last_joy_time_{0, 0, RCL_ROS_TIME}, last_status_time_{0, 0, RCL_ROS_TIME};
    std::chrono::milliseconds timeout_threshold_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArgusManagerNode>());
    rclcpp::shutdown();
    return 0;
}
