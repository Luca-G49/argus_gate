#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "argus_interfaces/msg/plc_command.hpp"
#include "argus_interfaces/msg/plc_status.hpp"

using namespace std::chrono_literals;

class PlcSimNode : public rclcpp::Node
{
public:
  PlcSimNode()
  : Node("plc_sim_node"),
    state_(PlcState::STARTUP),
    current_pitch_(0.0),
    current_yaw_(0.0),
    target_pitch_(0.0),
    target_yaw_(0.0),
    pitch_direction_(0),
    yaw_direction_(0),
    error_code_(0),
    sync_step_(0),
    busy_(false),
    done_(false),
    synch_(false),
    on_target_(false)
  {
    this->declare_parameter("publish_frequency_hz", 50.0);
    this->declare_parameter("step_per_cycle", 0.02);
    this->declare_parameter("target_tolerance", 0.01);
    this->declare_parameter("initial_pitch", 0.0);
    this->declare_parameter("initial_yaw", 0.0);

    double freq = this->get_parameter("publish_frequency_hz").as_double();
    step_per_cycle_ = this->get_parameter("step_per_cycle").as_double();
    target_tolerance_ = this->get_parameter("target_tolerance").as_double();
    current_pitch_ = this->get_parameter("initial_pitch").as_double();
    current_yaw_ = this->get_parameter("initial_yaw").as_double();

    status_pub_ = this->create_publisher<argus_interfaces::msg::PlcStatus>("plc_status", 10);
    command_sub_ = this->create_subscription<argus_interfaces::msg::PlcCommand>(
      "plc_command", 10,
      std::bind(&PlcSimNode::command_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / freq),
      std::bind(&PlcSimNode::publish_status, this));

    RCLCPP_INFO(this->get_logger(), "PLC simulator started on plc_status/plc_command");
  }

private:
  enum class PlcState : int8_t
  {
    STARTUP = 0,
    STOP = 1,
    READY = 2,
    SYNCH = 3,
    JOG = 4,
    FOLLOW = 5,
    ERROR = 6
  };

  void command_callback(const argus_interfaces::msg::PlcCommand::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_command_ = *msg;

    if (msg->fire && (state_ == PlcState::JOG || state_ == PlcState::FOLLOW)) {
      enter_error(100);
      return;
    }

    if (msg->ack && state_ == PlcState::ERROR) {
      clear_error();
      state_ = PlcState::READY;
      return;
    }

    if (msg->mode == 1 && msg->exec) {
      begin_sync();
      return;
    }

    if (msg->mode == 2) {
      state_ = PlcState::JOG;
      busy_ = false;
      done_ = false;
      synch_ = false;
      on_target_ = false;
      pitch_direction_ = msg->pitch_jog_p ? 1 : (msg->pitch_jog_n ? -1 : 0);
      yaw_direction_ = msg->yaw_jog_p ? 1 : (msg->yaw_jog_n ? -1 : 0);
      return;
    }

    if (msg->mode == 3) {
      if (msg->exec || state_ == PlcState::FOLLOW) {
        state_ = PlcState::FOLLOW;
        target_pitch_ = msg->target_pitch;
        target_yaw_ = msg->target_yaw;
        busy_ = true;
        done_ = false;
        synch_ = false;
        on_target_ = false;
      } else {
        state_ = PlcState::READY;
      }
      return;
    }

    state_ = PlcState::READY;
    busy_ = false;
    done_ = false;
    synch_ = false;
    on_target_ = false;
    pitch_direction_ = 0;
    yaw_direction_ = 0;
  }

  void publish_status()
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (state_ == PlcState::STARTUP) {
      state_ = PlcState::READY;
      busy_ = false;
      done_ = false;
      synch_ = false;
      on_target_ = false;
      error_code_ = 0;
      return;
    }

    if (state_ == PlcState::ERROR) {
      busy_ = false;
      done_ = false;
      synch_ = false;
      on_target_ = false;
    } else if (state_ == PlcState::SYNCH) {
      tick_sync();
    } else if (state_ == PlcState::JOG) {
      tick_jog();
    } else if (state_ == PlcState::FOLLOW) {
      tick_follow();
    } else {
      busy_ = false;
      done_ = false;
      on_target_ = false;
      if (state_ == PlcState::READY) {
        synch_ = false;
      }
    }

    argus_interfaces::msg::PlcStatus status;
    status.life_word = static_cast<uint16_t>((last_command_.life_word + 1) & 0xFFFF);
    status.done = done_;
    status.busy = busy_;
    status.synch = synch_;
    status.on_target = on_target_;
    status.status = static_cast<int16_t>(state_to_status_code(state_));
    status.error = error_code_;
    status.pos_pitch = static_cast<float>(current_pitch_);
    status.pos_yaw = static_cast<float>(current_yaw_);
    status.checksum = 0;

    status_pub_->publish(status);
  }

  void begin_sync()
  {
    state_ = PlcState::SYNCH;
    sync_step_ = 0;
    busy_ = true;
    done_ = false;
    synch_ = false;
    on_target_ = false;
  }

  void tick_sync()
  {
    bool pitch_done = false;
    bool yaw_done = false;

    if (sync_step_ == 0) {
      target_pitch_ = 0.0;
      pitch_done = move_towards(current_pitch_, target_pitch_);
      if (pitch_done) {
        ++sync_step_;
      }
    } else if (sync_step_ == 1) {
      target_yaw_ = 0.0;
      yaw_done = move_towards(current_yaw_, target_yaw_);
      if (yaw_done) {
        ++sync_step_;
      }
    } else {
      busy_ = false;
      done_ = true;
      synch_ = true;
      on_target_ = true;
      state_ = PlcState::READY;
      return;
    }

    busy_ = true;
    done_ = false;
    synch_ = false;
    on_target_ = false;
  }

  void tick_jog()
  {
    double pitch_scale = std::abs(last_command_.pitch_override) > 1e-6 ? last_command_.pitch_override : 1.0;
    double yaw_scale = std::abs(last_command_.yaw_override) > 1e-6 ? last_command_.yaw_override : 1.0;

    current_pitch_ += pitch_direction_ * step_per_cycle_ * pitch_scale;
    current_yaw_ += yaw_direction_ * step_per_cycle_ * yaw_scale;

    busy_ = false;
    done_ = false;
    synch_ = false;
    on_target_ = false;
  }

  void tick_follow()
  {
    bool pitch_done = move_towards(current_pitch_, target_pitch_);
    bool yaw_done = move_towards(current_yaw_, target_yaw_);

    busy_ = !(pitch_done && yaw_done);
    done_ = pitch_done && yaw_done;
    synch_ = false;
    on_target_ = done_;

    if (done_) {
      state_ = PlcState::READY;
    }
  }

  bool move_towards(double &current, double target)
  {
    if (std::abs(current - target) <= target_tolerance_) {
      current = target;
      return true;
    }

    double step = step_per_cycle_;
    if (current < target) {
      current = std::min(current + step, target);
    } else {
      current = std::max(current - step, target);
    }
    return false;
  }

  void enter_error(int error)
  {
    error_code_ = error;
    state_ = PlcState::ERROR;
    busy_ = false;
    done_ = false;
    synch_ = false;
    on_target_ = false;
  }

  void clear_error()
  {
    error_code_ = 0;
  }

  int state_to_status_code(PlcState state) const
  {
    switch (state) {
      case PlcState::STARTUP:
        return 0;
      case PlcState::STOP:
        return 1;
      case PlcState::READY:
        return 2;
      case PlcState::SYNCH:
        return 3;
      case PlcState::JOG:
        return 4;
      case PlcState::FOLLOW:
        return 5;
      case PlcState::ERROR:
      default:
        return -1;
    }
  }

  rclcpp::Publisher<argus_interfaces::msg::PlcStatus>::SharedPtr status_pub_;
  rclcpp::Subscription<argus_interfaces::msg::PlcCommand>::SharedPtr command_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex mutex_;
  argus_interfaces::msg::PlcCommand last_command_{};
  PlcState state_;
  double step_per_cycle_{0.02};
  double target_tolerance_{0.01};
  double current_pitch_{0.0};
  double current_yaw_{0.0};
  double target_pitch_{0.0};
  double target_yaw_{0.0};
  int pitch_direction_{0};
  int yaw_direction_{0};
  int error_code_{0};
  int sync_step_{0};
  bool busy_{false};
  bool done_{false};
  bool synch_{false};
  bool on_target_{false};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlcSimNode>());
  rclcpp::shutdown();
  return 0;
}
