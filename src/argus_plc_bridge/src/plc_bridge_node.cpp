#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <vector>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include <SFML/Network.hpp>
#include "argus_msgs/msg/plc_command.hpp"
#include "argus_msgs/msg/plc_status.hpp"

using namespace std::chrono_literals;

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
        status_pub_ = this->create_publisher<argus_msgs::msg::PlcStatus>("plc_status", 10);
        command_sub_ = this->create_subscription<argus_msgs::msg::PlcCommand>(
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
    void command_callback(const argus_msgs::msg::PlcCommand::SharedPtr msg) {
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

            // --- 1. TRANSMISSION (Request/Ping) ---
            std::string out_payload;
            {
                std::lock_guard<std::mutex> lock(mtx_);
                tx_data_.life_word = rx_data_.life_word; // Sync heartbeat for PLC watchdog
                out_payload = serialize_200(tx_data_);
            }
            socket_.send(out_payload.c_str(), out_payload.size(), plc_ip_, plc_port_);

            // --- 2. RECEPTION (Response/Pong) ---
            // Flush UDP buffer to retrieve only the most recent datagram
            char buffer[1024];
            std::size_t received;
            sf::IpAddress sender;
            unsigned short port;
            bool data_received = false;
            argus_msgs::msg::PlcStatus temp_status;

            while (socket_.receive(buffer, sizeof(buffer), received, sender, port) == sf::Socket::Done) {
                temp_status = deserialize_201(std::string(buffer, received));
                data_received = true;
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
     * @brief Serializes PlcCommand into the MSG_200 pipe-separated format.
     */
    std::string serialize_200(const argus_msgs::msg::PlcCommand& d) {
        std::stringstream ss;
        ss << "200|" << d.life_word << "|" 
           << d.ack << "|" << d.exec << "|" << d.fire << "|"
           << d.pitch_jog_p << "|" << d.pitch_jog_n << "|"
           << d.yaw_jog_p << "|" << d.yaw_jog_n << "|"
           << d.mode << "|" << d.pitch_override << "|" << d.yaw_override << "|"
           << d.target_pitch << "|" << d.target_yaw << "|" 
           << d.checksum;
        
        std::string content = ss.str();
        std::string header = "  ";
        header[0] = static_cast<char>(content.length() + 2); // Message length header
        header[1] = 0; 
        return header + content;
    }

    /**
     * @brief Parses the MSG_201 string into the PlcStatus structure.
     */
    argus_msgs::msg::PlcStatus deserialize_201(const std::string& raw) {
        argus_msgs::msg::PlcStatus s;
        if (raw.length() < 5) return s;

        std::string payload = raw.substr(2); // Skip binary length header
        std::stringstream ss(payload);
        std::string item;
        std::vector<std::string> v;

        while (std::getline(ss, item, '|')) v.push_back(item);

        try {
            if (v.size() >= 11 && v[0] == "201") {
                s.life_word   = std::stoi(v[1]);
                s.done        = (v[2] == "1");
                s.busy        = (v[3] == "1");
                s.synch       = (v[4] == "1");
                s.on_target   = (v[5] == "1");
                s.status      = std::stoi(v[6]);
                s.error       = std::stoi(v[7]);
                s.pos_pitch   = std::stol(v[8]);
                s.pos_yaw     = std::stol(v[9]);
                s.checksum    = std::stoi(v[10]);
            }
        } catch (...) {
            // Return default/zeroed status on parsing error
        }
        return s;
    }

    /**
     * @brief Checks if a watchdog timer is still valid
     */
    bool check_watchdog(const rclcpp::Time &last_time) {
        return (this->get_clock()->now() - last_time) < timeout_threshold_;
    }

    // ROS2 Members
    rclcpp::Publisher<argus_msgs::msg::PlcStatus>::SharedPtr status_pub_;
    rclcpp::Subscription<argus_msgs::msg::PlcCommand>::SharedPtr command_sub_;

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

    argus_msgs::msg::PlcCommand tx_data_;
    argus_msgs::msg::PlcStatus rx_data_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PlcBridgeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
