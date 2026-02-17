#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <vector>
#include <cstring>

#include "rclcpp/rclcpp.hpp"
#include <SFML/Network.hpp>
#include "argus_interfaces/msg/plc_command.hpp"
#include "argus_interfaces/msg/plc_status.hpp"

using namespace std::chrono_literals;

// --- BINARY STRUCTURES FOR DIRECT MEMORY MAPPING ---
#pragma pack(push, 1)
struct RawTx200 {
    uint16_t id = 200;           // Message ID
    uint16_t life_word;          // Heartbeat
    uint8_t  flags;              // bool ack, exec, fire (bit 0,1,2)
    uint8_t  jogs;               // bool pitch_jog_p, n, yaw_jog_p, n (bit 0,1,2,3)
    int16_t  mode;               
    float    pitch_override;     
    float    yaw_override;       
    float    target_pitch;       
    float    target_yaw;         
    uint16_t checksum;           
};

struct RawRx201 {
    uint16_t id;                 // Message ID from PLC
    uint16_t life_word;          
    uint8_t  flags;              // bool done, busy, synch, on_target
    int16_t  status;             
    int16_t  error;              
    float    pos_pitch;          
    float    pos_yaw;            
    uint16_t checksum;           
};
#pragma pack(pop)

/**
 * @class PlcBridgeNode
 * @brief Handles asynchronous UDP communication with the PLC at a fixed frequency.
 */
class PlcBridgeNode : public rclcpp::Node {
public:
    PlcBridgeNode() : Node("plc_bridge_node") {
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
    ~PlcBridgeNode() {
        running_ = false;
        if (net_thread_.joinable()) net_thread_.join();
    }

private:
    /**
     * @brief Thread-safe update of motion control targets from ROS2 topic.
     */
    void command_callback(const argus_interfaces::msg::PlcCommand::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mtx_);
        tx_data_ = *msg;
    }

    /**
     * @brief Main worker loop: executes at fixed frequency.
     */
    void network_loop(int freq) {
        auto interval = std::chrono::milliseconds(1000 / freq);
        
        while (running_ && rclcpp::ok()) {
            auto next_cycle = std::chrono::steady_clock::now() + interval;

            // --- 1. TRANSMISSION (Binary Struct) ---
            RawTx200 out_packet;
            {
                std::lock_guard<std::mutex> lock(mtx_);
                tx_data_.life_word = rx_data_.life_word; // Sync heartbeat for PLC watchdog
                out_packet = serialize_200(tx_data_);
            }
            socket_.send(&out_packet, sizeof(out_packet), plc_ip_, plc_port_);

            // --- 2. RECEPTION (Binary Struct) ---
            // Flush UDP buffer to retrieve only the most recent datagram
            RawRx201 in_raw;
            std::size_t received;
            sf::IpAddress sender;
            unsigned short port;
            bool data_received = false;
            argus_interfaces::msg::PlcStatus temp_status;

            while (socket_.receive(&in_raw, sizeof(in_raw), received, sender, port) == sf::Socket::Done) {
                // Check if size is correct and ID matches
                if (received == sizeof(RawRx201) && in_raw.id == 201) {
                    temp_status = deserialize_201(in_raw);
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
     * @brief Serializes PlcCommand into the RawTx200 binary format.
     */
    RawTx200 serialize_200(const argus_interfaces::msg::PlcCommand& d) {
        RawTx200 p {};
        p.life_word = d.life_word;
        // Pack bools into bits
        p.flags = (d.ack << 0) | (d.exec << 1) | (d.fire << 2);
        p.jogs  = (d.pitch_jog_p << 0) | (d.pitch_jog_n << 1) | (d.yaw_jog_p << 2) | (d.yaw_jog_n << 3);
        p.mode = d.mode;
        p.pitch_override = d.pitch_override;
        p.yaw_override = d.yaw_override;
        p.target_pitch = d.target_pitch;
        p.target_yaw = d.target_yaw;
        p.checksum = d.checksum;
        return p;
    }

    /**
     * @brief Parses the RawRx201 binary struct into the PlcStatus structure.
     */
    argus_interfaces::msg::PlcStatus deserialize_201(const RawRx201& raw) {
        argus_interfaces::msg::PlcStatus s;
        s.life_word = raw.life_word;
        // Unpack bits into bools
        s.done      = (raw.flags >> 0) & 1;
        s.busy      = (raw.flags >> 1) & 1;
        s.synch     = (raw.flags >> 2) & 1;
        s.on_target = (raw.flags >> 3) & 1;
        s.status    = raw.status;
        s.error     = raw.error;
        s.pos_pitch = raw.pos_pitch;
        s.pos_yaw   = raw.pos_yaw;
        s.checksum  = raw.checksum;
        return s;
    }

    /**
     * @brief Checks if a watchdog timer is still valid
     */
    bool check_watchdog(const rclcpp::Time &last_time) {
        return (this->get_clock()->now() - last_time) < timeout_threshold_;
    }

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

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PlcBridgeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
