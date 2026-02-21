#include <memory>
#include <chrono>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include "argus_interfaces/msg/teleop_command.hpp"
#include "argus_interfaces/msg/plc_command.hpp"
#include "argus_interfaces/msg/plc_status.hpp"

using namespace std::chrono_literals;

/**
 * @class ArgusManagerNode
 * @brief Logic master: arbitrates system modes and ensures hardware safety interlocks.
 */
class ArgusManagerNode : public rclcpp::Node {
public:
    // Modes mandated by ROS2 to the PLC
    enum class ControlMode : int16_t  {
        IDLE = 0,
        MANUAL_JOG = 1,
        MANUAL_TRACK = 2,
        AUTO_TRACK = 5,
        ERROR_RESET = 99
    };

    ArgusManagerNode() : Node("argus_manager_node"), requested_mode_(ControlMode::IDLE) {
        // Initialize watchdog timers to "now"
        last_teleop_time_ = this->get_clock()->now();
        last_status_time_ = this->get_clock()->now();

        // --- 1. PARAMETERS ---
        this->declare_parameter("watchdog_timeout_ms", 500);
        this->declare_parameter("loop_frequency", 50.0);
        
        timeout_threshold_ = std::chrono::milliseconds(this->get_parameter("watchdog_timeout_ms").as_int());
        double freq = this->get_parameter("loop_frequency").as_double();

        // --- 2. COMMS ---
        teleop_sub_  = this->create_subscription<argus_interfaces::msg::TeleopCommand>(
            "teleop_cmd", 10, std::bind(&ArgusManagerNode::on_teleop_received, this, std::placeholders::_1));
        
        status_sub_  = this->create_subscription<argus_interfaces::msg::PlcStatus>(
            "plc_status", 10, std::bind(&ArgusManagerNode::on_status_received, this, std::placeholders::_1));
        
        command_pub_ = this->create_publisher<argus_interfaces::msg::PlcCommand>("plc_command", 10);

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
        argus_interfaces::msg::PlcCommand outbound_cmd;
        reset_command(outbound_cmd);

        // --- STEP 1: QOS / HEALTH MONITORING ---
        bool teleop_alive = check_watchdog(last_teleop_time_);
        bool plc_alive = check_watchdog(last_status_time_);

        if (!plc_alive) {
            log_fault("PLC Link Lost: Inhibiting commands", 2000);
            return; 
        }

        // --- STEP 2: MODE ARBITRATION ---
        // Update the requested mode based on UI/Teleop inputs
        update_requested_mode(teleop_alive);

        // Force IDLE mode if hardware reports an error
        if (last_status_.error != 0) {
            requested_mode_ = ControlMode::IDLE;
            process_error_recovery(outbound_cmd);
        } 
        else if (!teleop_alive && requested_mode_ == ControlMode::MANUAL_JOG) {
            log_warn("Teleop Offline: Forcing Safe Stop", 2000);
            requested_mode_ = ControlMode::IDLE;
        } 
        else {
            // --- STEP 3: LOGIC EXECUTION ---
            execute_mode_logic(outbound_cmd);
        }

        // Set the final mode mandated by ROS2
        outbound_cmd.mode = static_cast<int16_t>(requested_mode_);
        command_pub_->publish(outbound_cmd);
    }

    /**
     * @brief Logic to switch between control modes.
     */
    void update_requested_mode(bool teleop_ready) {
        if (!teleop_ready) return;

        // Mode switching via Teleop requests
        if (last_teleop_.requested_mode == 0) requested_mode_ = ControlMode::IDLE;
        if (last_teleop_.requested_mode == 1) requested_mode_ = ControlMode::MANUAL_JOG;
        if (last_teleop_.requested_mode == 2) requested_mode_ = ControlMode::MANUAL_TRACK;
    }

    /**
     * @brief Handles motion generation based on the active requested mode.
     */
    void execute_mode_logic(argus_interfaces::msg::PlcCommand &cmd) {
        switch (requested_mode_) {
            case ControlMode::MANUAL_JOG:
                map_teleop_to_jog(cmd);
                break;
            
            case ControlMode::MANUAL_TRACK:
                if (last_teleop_.send_target) {
                    cmd.target_pitch = last_teleop_.target_pitch;
                    cmd.target_yaw   = last_teleop_.target_yaw;
                    cmd.exec = true; // Signal PLC to move to target
                }
                break;

            default:
                break;
        }
    }

    /**
     * @brief Maps abstracted teleop speeds to PLC Jog commands.
     */
    void map_teleop_to_jog(argus_interfaces::msg::PlcCommand &cmd) {
        const float deadzone = 0.15f;

        float p = last_teleop_.pitch_speed;
        cmd.pitch_jog_p = (p >  deadzone);
        cmd.pitch_jog_n = (p < -deadzone);
        cmd.pitch_override = std::abs(p) * 100.0f;

        float y = last_teleop_.yaw_speed;
        cmd.yaw_jog_p = (y >  deadzone);
        cmd.yaw_jog_n = (y < -deadzone);
        cmd.yaw_override = std::abs(y) * 100.0f;
        
        cmd.ack  = last_teleop_.reset_error;
        cmd.exec = last_teleop_.execute_action;
    }

    void process_error_recovery(argus_interfaces::msg::PlcCommand &cmd) {
        cmd.ack = last_teleop_.reset_error;
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "PLC ERROR: Manual reset required (ACK)");
    }

    // --- HELPER FUNCTIONS ---
    
    bool check_watchdog(const rclcpp::Time &last_time) {
        return (this->get_clock()->now() - last_time) < timeout_threshold_;
    }

    void reset_command(argus_interfaces::msg::PlcCommand &cmd) {
        cmd.pitch_jog_p = cmd.pitch_jog_n = cmd.yaw_jog_p = cmd.yaw_jog_n = false;
        cmd.ack = cmd.exec = cmd.fire = false;
        cmd.pitch_override = cmd.yaw_override = 0.0f;
        cmd.target_pitch = cmd.target_yaw = 0.0f;
        cmd.mode = static_cast<int16_t>(ControlMode::IDLE);
    }

    void log_fault(const std::string &msg, int throttle_ms) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), throttle_ms, "%s", msg.c_str());
    }

    void log_warn(const std::string &msg, int throttle_ms) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), throttle_ms, "%s", msg.c_str());
    }

    void on_teleop_received(const argus_interfaces::msg::TeleopCommand::SharedPtr msg) {
        last_teleop_ = *msg;
        last_teleop_time_ = this->get_clock()->now();
    }

    void on_status_received(const argus_interfaces::msg::PlcStatus::SharedPtr msg) {
        last_status_ = *msg;
        last_status_time_ = this->get_clock()->now();
    }

    // Members
    rclcpp::Subscription<argus_interfaces::msg::TeleopCommand>::SharedPtr teleop_sub_;
    rclcpp::Subscription<argus_interfaces::msg::PlcStatus>::SharedPtr status_sub_;
    rclcpp::Publisher<argus_interfaces::msg::PlcCommand>::SharedPtr command_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    ControlMode requested_mode_;
    argus_interfaces::msg::TeleopCommand last_teleop_;
    argus_interfaces::msg::PlcStatus last_status_;
    rclcpp::Time last_teleop_time_, last_status_time_;
    std::chrono::milliseconds timeout_threshold_{500};
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArgusManagerNode>());
    rclcpp::shutdown();
    return 0;
}
