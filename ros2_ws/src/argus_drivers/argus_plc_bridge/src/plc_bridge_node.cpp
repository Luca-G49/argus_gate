/*
 * Argus Gate - argus_plc_bridge/src/plc_bridge_node.cpp
 * Copyright (c) 2026, Name
 * All rights reserved. Licensed under MIT License.
 */

#include "plc_bridge_node.hpp"
#include "packet_utils.hpp"

using namespace std::chrono_literals;

PlcBridgeNode::PlcBridgeNode() : Node("plc_bridge_node") {
    // Initialize watchdog timers to "now"
    last_rx_time_ = this->get_clock()->now();

    // --- 1. PARAMETERS DECLARATION ---
    this->declare_parameter("plc_ip", "127.0.0.1");
    this->declare_parameter("remote_port", 55753);
    this->declare_parameter("local_port", 48585);
    this->declare_parameter("frequency", 100);

        std::string ip = this->get_parameter("plc_ip").as_string();
        int r_port = this->get_parameter("remote_port").as_int();
        int l_port = this->get_parameter("local_port").as_int();
        int freq = this->get_parameter("frequency").as_int();

        // --- 2. NETWORK SETUP ---
        if (socket_.bind(l_port) != sf::Socket::Done) {
            RCLCPP_ERROR(this->get_logger(), "Failed to bind local UDP port %d", l_port);
            throw std::runtime_error("Failed to bind local UDP port.");
        }
        // Enable non-blocking mode for high-frequency polling
        socket_.setBlocking(false);
        plc_ip_ = ip;
        plc_port_ = static_cast<unsigned short>(r_port);

        // --- 3. ROS2 PUBS/SUBS ---
        status_pub_ = this->create_publisher<argus_interfaces::msg::PlcStatus>("plc_status", 10);
        command_sub_ = this->create_subscription<argus_interfaces::msg::PlcCommand>(
            "plc_command", 10, std::bind(&PlcBridgeNode::command_callback, this, std::placeholders::_1));

        // --- 4. START NETWORK THREAD ---
        // Spawns the dedicated network communication thread
        running_ = true;
        net_thread_ = std::thread(&PlcBridgeNode::network_loop, this, freq);
        
        RCLCPP_INFO(this->get_logger(), "--- PLC Bridge Active --- Target: %s:%d", ip.c_str(), r_port);
    }

    /**
     * @brief Signals the thread to stop and joins it.
     */
    PlcBridgeNode::~PlcBridgeNode() {
        running_ = false;
        if (net_thread_.joinable()) net_thread_.join();
    }

    /**
     * @brief Thread-safe update of motion control targets from ROS2 topic.
     */
    void PlcBridgeNode::command_callback(const argus_interfaces::msg::PlcCommand::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mtx_);
        tx_data_ = *msg;
    }

    /**
     * @brief Main worker loop: executes at fixed frequency.
     */
    void PlcBridgeNode::network_loop(int freq) {
        auto interval = std::chrono::milliseconds(1000 / freq);
        
        while (running_ && rclcpp::ok()) {
            auto next_cycle = std::chrono::steady_clock::now() + interval;

            // --- 1. TRANSMISSION (Binary Struct) ---
            RawTxData out_packet;
            {
                std::lock_guard<std::mutex> lock(mtx_);
                tx_data_.life_word = rx_data_.life_word; // Sync heartbeat for PLC watchdog
                out_packet = encode_packet(tx_data_);
            }
            socket_.send(&out_packet, sizeof(out_packet), plc_ip_, plc_port_);

            // --- 2. RECEPTION (Binary Struct) ---
            // Flush UDP buffer to retrieve only the most recent datagram
            RawRxData in_raw;
            std::size_t received;
            sf::IpAddress sender;
            unsigned short port;
            bool data_received = false;
            argus_interfaces::msg::PlcStatus temp_status;

            while (socket_.receive(&in_raw, sizeof(in_raw), received, sender, port) == sf::Socket::Done) {
                // Check if size is correct and ID matches
                if (received == sizeof(RawRxData) && ntohs(in_raw.id) == 201) {
                    temp_status = decode_packet(in_raw);
                    data_received = true;
                }
            }

            if (data_received) {
                // Publish only when new data is received
                {
                    std::lock_guard<std::mutex> lock(mtx_);
                    rx_data_ = temp_status;
                }
                // Update the watchdog timer on successful reception
                last_rx_time_ = this->get_clock()->now();
                // Publish the new status to ROS2
                status_pub_->publish(temp_status);
            } else {
                // Log a warning if no data is received for a while
                if (!check_watchdog(last_rx_time_)) {
                    auto silence = (this->get_clock()->now() - last_rx_time_).seconds() * 1000.0;
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                                        "No data from PLC for %.1f ms...", silence);
                }
            }

            // --- 3. DETERMINISTIC TIMING ---
            std::this_thread::sleep_until(next_cycle);
        }
    }

    /**
     * @brief Checks if a watchdog timer is still valid
     */
    bool PlcBridgeNode::check_watchdog(const rclcpp::Time &last_time) {
        return (this->get_clock()->now() - last_time) < timeout_threshold_;
    }

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PlcBridgeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
