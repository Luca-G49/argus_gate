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
 * @brief Logic master: arbitrates system modes and ensures hardware safety interlocks.
 */
class ArgusManagerNode : public rclcpp::Node {
public:
    // Modes mandated by ROS2 to the PLC
    enum class ControlMode : int32_t {
        IDLE = 1,
        MANUAL_JOG = 4,
        AUTO_TRACK = 5,
        ERROR_RESET = 99
    };

    ArgusManagerNode() : Node("argus_manager_node"), requested_mode_(ControlMode::IDLE) {
        // Initialize watchdog timers to "now"
        last_joy_time_ = this->get_clock()->now();
        last_status_time_ = this->get_clock()->now();

        // --- 1. PARAMETERS ---
        this->declare_parameter("watchdog_timeout_ms", 500);
        this->declare_parameter("loop_frequency", 50.0);
        
        timeout_threshold_ = std::chrono::milliseconds(this->get_parameter("watchdog_timeout_ms").as_int());
        double freq = this->get_parameter("loop_frequency").as_double();

        // --- 2. COMMS ---
        joy_sub_     = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&ArgusManagerNode::on_joy_received, this, std::placeholders::_1));
        status_sub_  = this->create_subscription<argus_msgs::msg::PlcStatus>("plc_status", 10, std::bind(&ArgusManagerNode::on_status_received, this, std::placeholders::_1));
        command_pub_ = this->create_publisher<argus_msgs::msg::PlcCommand>("plc_command", 10);

        // --- 3. MAIN CYCLE (50Hz) ---
        auto interval = std::chrono::duration<double>(1.0 / freq);
        timer_ = this->create_wall_timer(interval, std::bind(&ArgusManagerNode::supervisor_cycle, this));

        RCLCPP_INFO(this->get_logger(), "--- Argus Manager Active --- Target Freq: %.1f Hz", freq);
    }

private:
    /**
     * @brief Supervisor loop: Health Check -> Mode Arbitration -> Command Dispatch.
     */
    void supervisor_cycle() {
        argus_msgs::msg::PlcCommand outbound_cmd;
        reset_command(outbound_cmd);

        // --- STEP 1: QOS / HEALTH MONITORING ---
        bool joy_alive = check_watchdog(last_joy_time_);
        bool plc_alive = check_watchdog(last_status_time_);

        if (!plc_alive) {
            log_fault("PLC Link Lost: Inhibiting commands", 2000);
            return; 
        }

        // --- STEP 2: MODE ARBITRATION ---
        // Update the requested mode based on UI/Joystick inputs
        update_requested_mode(joy_alive);

        // Force IDLE mode if hardware reports an error
        if (last_status_.error != 0) {
            requested_mode_ = ControlMode::IDLE;
            process_error_recovery(outbound_cmd);
        } 
        else if (!joy_alive && requested_mode_ == ControlMode::MANUAL_JOG) {
            log_warn("Joystick Offline: Forcing Safe Stop", 2000);
            requested_mode_ = ControlMode::IDLE;
        } 
        else {
            // --- STEP 3: LOGIC EXECUTION ---
            execute_mode_logic(outbound_cmd);
        }

        // Set the final mode mandated by ROS2
        outbound_cmd.mode = static_cast<int32_t>(requested_mode_);
        command_pub_->publish(outbound_cmd);
    }

    /**
     * @brief Logic to switch between control modes.
     */
    void update_requested_mode(bool joy_ready) {
        if (!joy_ready || last_joy_.buttons.size() < 4) return;

        // Mode switching via Joystick buttons (Example: PS4 layout)
        if (last_joy_.buttons[3]) requested_mode_ = ControlMode::MANUAL_JOG; // Triangle -> Manual
        if (last_joy_.buttons[2]) requested_mode_ = ControlMode::IDLE;       // Square -> Idle
    }

    /**
     * @brief Handles motion generation based on the active requested mode.
     */
    void execute_mode_logic(argus_msgs::msg::PlcCommand &cmd) {
        switch (requested_mode_) {
            case ControlMode::MANUAL_JOG:
                map_joystick_to_jog(cmd);
                break;
            
            case ControlMode::AUTO_TRACK:
                // Future: Tracking logic from Vision node
                break;

            default:
                break;
        }
    }

    void map_joystick_to_jog(argus_msgs::msg::PlcCommand &cmd) {
        if (last_joy_.axes.size() < 8) return;

        const float deadzone = 0.15f;
        
        // Pitch (Stick LY or D-Pad V)
        cmd.pitch_jog_p = (last_joy_.axes[1] > deadzone  || last_joy_.axes[7] > 0.5f);
        cmd.pitch_jog_n = (last_joy_.axes[1] < -deadzone || last_joy_.axes[7] < -0.5f);

        // Yaw (Stick LX or D-Pad H)
        cmd.yaw_jog_p   = (last_joy_.axes[0] > deadzone  || last_joy_.axes[6] > 0.5f);
        cmd.yaw_jog_n   = (last_joy_.axes[0] < -deadzone || last_joy_.axes[6] < -0.5f);

        cmd.pitch_override = 100;
        cmd.yaw_override   = 100;
        
        if (last_joy_.buttons.size() >= 2) {
            cmd.ack  = last_joy_.buttons[0]; // Cross
            cmd.exec = last_joy_.buttons[1]; // Circle
        }
    }

    void process_error_recovery(argus_msgs::msg::PlcCommand &cmd) {
        if (!last_joy_.buttons.empty()) {
            cmd.ack = last_joy_.buttons[0];
        }
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "PLC ERROR: Manual reset required (ACK)");
    }

    // --- HELPER FUNCTIONS ---
    
    bool check_watchdog(const rclcpp::Time &last_time) {
        return (this->get_clock()->now() - last_time) < timeout_threshold_;
    }

    void reset_command(argus_msgs::msg::PlcCommand &cmd) {
        cmd.pitch_jog_p = cmd.pitch_jog_n = cmd.yaw_jog_p = cmd.yaw_jog_n = false;
        cmd.ack = cmd.exec = cmd.fire = false;
        cmd.pitch_override = cmd.yaw_override = 0;
        cmd.mode = static_cast<int32_t>(ControlMode::IDLE);
    }

    void log_fault(const std::string &msg, int throttle_ms) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), throttle_ms, "%s", msg.c_str());
    }

    void log_warn(const std::string &msg, int throttle_ms) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), throttle_ms, "%s", msg.c_str());
    }

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

    ControlMode requested_mode_;
    sensor_msgs::msg::Joy last_joy_;
    argus_msgs::msg::PlcStatus last_status_;
    rclcpp::Time last_joy_time_, last_status_time_;
    std::chrono::milliseconds timeout_threshold_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArgusManagerNode>());
    rclcpp::shutdown();
    return 0;
}
