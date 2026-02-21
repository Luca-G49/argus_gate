/*
 * Argus Gate - argus_plc_bridge/src/plc_bridge_node.hpp
 * Copyright (c) 2026, Luca-G49
 * All rights reserved. Licensed under MIT License.
 */

#ifndef PLC_BRIDGE_NODE_HPP
#define PLC_BRIDGE_NODE_HPP

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <vector>
#include <cstring>
#include <arpa/inet.h>
#include <SFML/Network.hpp>

#include "rclcpp/rclcpp.hpp"
#include "argus_interfaces/msg/plc_command.hpp"
#include "argus_interfaces/msg/plc_status.hpp"

using namespace std::chrono_literals;

// --- BINARY STRUCTURES FOR DIRECT MEMORY MAPPING ---
#pragma pack(push, 1)
struct RawTxData {
    uint16_t id = 200;           // Offset 0
    uint16_t life_word;          // Offset 2
    uint8_t  flags;              // Offset 4 (ack, exec, fire)
    uint8_t  jogs;               // Offset 5 (pitch_p/n, yaw_p/n)
    int16_t  mode;               // Offset 6
    int32_t  pitch_override;     // Offset 8  (DINT scaled x100)
    int32_t  yaw_override;       // Offset 12 (DINT scaled x100)
    int32_t  target_pitch;       // Offset 16 (DINT scaled x100)
    int32_t  target_yaw;         // Offset 20 (DINT scaled x100)
    uint16_t checksum;           // Offset 24
};

struct RawRxData {
    uint16_t id;                 // Offset 0
    uint16_t life_word;          // Offset 2
    uint8_t  flags;              // Offset 4 (done, busy, synch, on_target)
    uint8_t  reserved;           // Offset 5 (Padding for alignment)
    int16_t  status;             // Offset 6
    int16_t  error;              // Offset 8
    int32_t  pos_pitch;          // Offset 10 (DINT scaled x100)
    int32_t  pos_yaw;            // Offset 14 (DINT scaled x100)
    uint16_t checksum;           // Offset 18
};
#pragma pack(pop)

/**
 * @class PlcBridgeNode
 * @brief Handles asynchronous UDP communication with the PLC at a fixed frequency.
 */
class PlcBridgeNode : public rclcpp::Node {
public:
    PlcBridgeNode();
    ~PlcBridgeNode();

private:
    void command_callback(const argus_interfaces::msg::PlcCommand::SharedPtr msg);
    void network_loop(int freq);
    bool check_watchdog(const rclcpp::Time &last_time);

    // ROS2 Members
    rclcpp::Publisher<argus_interfaces::msg::PlcStatus>::SharedPtr status_pub_;
    rclcpp::Subscription<argus_interfaces::msg::PlcCommand>::SharedPtr command_sub_;

    // Watchdog timer for PLC communication
    rclcpp::Time last_rx_time_{0, 0, RCL_ROS_TIME};
    std::chrono::milliseconds timeout_threshold_{500};

    // Network Members
    sf::UdpSocket socket_;
    sf::IpAddress plc_ip_;
    unsigned short plc_port_;

    std::atomic<bool> running_{false};
    std::thread net_thread_;
    std::mutex mtx_;

    argus_interfaces::msg::PlcCommand tx_data_;
    argus_interfaces::msg::PlcStatus rx_data_;
};

#endif // PLC_BRIDGE_NODE_HPP