#include <chrono>
#include <cstdio>
#include <iostream>
#include <memory>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <unordered_map>
#include <sys/select.h>

#include "rclcpp/rclcpp.hpp"
#include "argus_interfaces/msg/teleop_command.hpp"

using namespace std::chrono_literals;

class TeleopKeyboardNode : public rclcpp::Node
{
public:
  TeleopKeyboardNode()
  : Node("argus_teleop_keyboard_node")
  {
    this->declare_parameter("publish_rate_hz", 50.0);
    this->declare_parameter("release_timeout_ms", 500);
    this->declare_parameter("target_step", 0.1);

    publish_rate_hz_ = this->get_parameter("publish_rate_hz").as_double();
    release_timeout_ms_ = this->get_parameter("release_timeout_ms").as_int();
    target_step_ = this->get_parameter("target_step").as_double();

    teleop_pub_ = this->create_publisher<argus_interfaces::msg::TeleopCommand>("teleop_cmd", 10);
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate_hz_),
      std::bind(&TeleopKeyboardNode::publish_loop, this));

    setup_terminal();
    RCLCPP_INFO(this->get_logger(),
                "Keyboard teleop active. Modes: 0 idle, 1 synch, 2 jog, 3 follow. "
                "Jog: W/S pitch, A/D yaw. Follow targets: I/K pitch, J/L yaw. "
                "Exec: Space, Send target: T, Reset: R");
  }

  ~TeleopKeyboardNode() override
  {
    restore_terminal();
  }

private:
  void setup_terminal()
  {
    tcgetattr(STDIN_FILENO, &old_tio_);
    tty_ = old_tio_;
    tty_.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &tty_);
  }

  void restore_terminal()
  {
    tcsetattr(STDIN_FILENO, TCSANOW, &old_tio_);
  }

  bool read_key_if_available(char &c)
  {
    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(STDIN_FILENO, &readfds);

    timeval timeout{0, 0};
    int ready = select(STDIN_FILENO + 1, &readfds, nullptr, nullptr, &timeout);
    if (ready <= 0) {
      return false;
    }

    return read(STDIN_FILENO, &c, 1) > 0;
  }

  void refresh_pressed_keys()
  {
    auto now = std::chrono::steady_clock::now();
    auto timeout = std::chrono::milliseconds(release_timeout_ms_);

    for (auto it = key_timestamps_.begin(); it != key_timestamps_.end();) {
      if (now - it->second > timeout) {
        pressed_keys_.erase(it->first);
        it = key_timestamps_.erase(it);
      } else {
        ++it;
      }
    }
  }

  void publish_loop()
  {
    char c = 0;
    while (read_key_if_available(c)) {
      handle_key(c);
    }

    refresh_pressed_keys();

    if (pressed_keys_.count('w') || pressed_keys_.count('W')) {
      pitch_speed_ = 1.0f;
    } else if (pressed_keys_.count('s') || pressed_keys_.count('S')) {
      pitch_speed_ = -1.0f;
    } else {
      pitch_speed_ = 0.0f;
    }

    if (pressed_keys_.count('d') || pressed_keys_.count('D')) {
      yaw_speed_ = 1.0f;
    } else if (pressed_keys_.count('a') || pressed_keys_.count('A')) {
      yaw_speed_ = -1.0f;
    } else {
      yaw_speed_ = 0.0f;
    }

    auto msg = argus_interfaces::msg::TeleopCommand();
    msg.pitch_speed = pitch_speed_;
    msg.yaw_speed = yaw_speed_;
    msg.target_pitch = target_pitch_;
    msg.target_yaw = target_yaw_;
    msg.requested_mode = requested_mode_;
    msg.execute_action = execute_action_;
    msg.reset_error = reset_error_;
    msg.send_target = send_target_;

    teleop_pub_->publish(msg);

    execute_action_ = false;
    reset_error_ = false;
    send_target_ = false;
  }

  void handle_key(char c)
  {
    auto now = std::chrono::steady_clock::now();

    switch (c) {
      case 'w':
      case 'W':
      case 's':
      case 'S':
      case 'a':
      case 'A':
      case 'd':
      case 'D':
        pressed_keys_[c] = true;
        key_timestamps_[c] = now;
        break;

      case 'i':
      case 'I':
        target_pitch_ += target_step_;
        break;
      case 'k':
      case 'K':
        target_pitch_ -= target_step_;
        break;
      case 'j':
      case 'J':
        target_yaw_ -= target_step_;
        break;
      case 'l':
      case 'L':
        target_yaw_ += target_step_;
        break;

      case '0':
        requested_mode_ = 0;
        break;
      case '1':
        requested_mode_ = 1;
        break;
      case '2':
        requested_mode_ = 2;
        break;
      case '3':
        requested_mode_ = 3;
        break;
      case ' ':
        execute_action_ = true;
        break;
      case 't':
      case 'T':
        send_target_ = true;
        break;
      case 'r':
      case 'R':
        reset_error_ = true;
        break;
      default:
        break;
    }
  }

  rclcpp::Publisher<argus_interfaces::msg::TeleopCommand>::SharedPtr teleop_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  struct termios old_tio_{};
  struct termios tty_{};

  std::unordered_map<char, bool> pressed_keys_;
  std::unordered_map<char, std::chrono::steady_clock::time_point> key_timestamps_;

  double publish_rate_hz_{50.0};
  int release_timeout_ms_{120};
  double target_step_{0.1};

  float pitch_speed_{0.0f};
  float yaw_speed_{0.0f};
  float target_pitch_{0.0f};
  float target_yaw_{0.0f};
  int16_t requested_mode_{0};
  bool execute_action_{false};
  bool reset_error_{false};
  bool send_target_{false};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TeleopKeyboardNode>());
  rclcpp::shutdown();
  return 0;
}
