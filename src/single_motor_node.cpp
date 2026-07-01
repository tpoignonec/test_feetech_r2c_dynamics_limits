// Standalone single-motor node.
//
// Mirrors the FeetechHardwareInterface configuration/activation flow for a
// *single* motor, then runs a simple open/close position demo. It is meant as
// a small, self-contained way to bring up one servo (e.g. the gripper) without
// spinning up ros2_control.
//
// Flow:
//   1. Open the serial port and configure the motor from joint_config.yaml
//      (PID, angle limits, homing offset, max_torque_limit, ...).
//   2. Activate the motor (enable torque).
//   3. Read back the EPROM parameters (max_torque_limit in particular) and log.
//   4. write_position -> open position, wait a couple seconds.
//   5. write_position -> close position.
//
// open/close position, max velocity and max force (max_torque_limit) are ROS
// parameters, so they can be tuned from a launch file / CLI without touching
// the YAML.

#include <algorithm>
#include <chrono>
#include <experimental/array>
#include <feetech_driver/SMS_STS.h>
#include <feetech_driver/common.hpp>
#include <feetech_driver/communication_protocol.hpp>
#include <feetech_driver/serial_port.hpp>
#include <feetech_ros2_driver/joint_config.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <thread>

using namespace std::chrono_literals;

namespace feetech_ros2_driver {

class SingleMotorNode : public rclcpp::Node {
 public:
  SingleMotorNode() : rclcpp::Node("feetech_single_motor") {
    // Transport / config
    usb_port_ = declare_parameter<std::string>("usb_port", "/dev/ttyACM0");
    joint_config_file_ = declare_parameter<std::string>("joint_config_file", "");
    joint_name_ = declare_parameter<std::string>("joint_name", "gripper");

    // Motion parameters (ROS params so they can be tuned without the YAML)
    // Positions are in radians, centered on the servo midpoint (0 rad = 2048 ticks),
    // matching the FeetechHardwareInterface position convention.
    open_position_rad_ = declare_parameter<double>("open_position", 0.0);
    close_position_rad_ = declare_parameter<double>("close_position", 2.2);
    // Default limits used both for configuration (EPROM max_torque_limit) and
    // as the values applied dynamically on every write_position command.
    // Goal speed in rad/s (converted to servo ticks/s below).
    max_velocity_rad_s_ = declare_parameter<double>("max_velocity", 3.0);
    // max torque limit as a percentage of rated torque (0..100 %). Internally
    // the servo register is 0..1000 (unit 0.1%), so this is scaled by 10.
    // -1 => keep the value coming from joint_config.yaml.
    max_torque_percent_ = declare_parameter<double>("max_torque", -1.0);
    acceleration_ = static_cast<int>(declare_parameter<int64_t>("acceleration", 50));
    wait_seconds_ = declare_parameter<double>("wait_seconds", 2.0);
    // Longer pause between the two open/close cycles.
    cycle_wait_seconds_ = declare_parameter<double>("cycle_wait_seconds", 5.0);
  }

  // Returns true on success. Runs the whole one-shot sequence.
  bool run() {
    if (!init_transport_()) {
      return false;
    }
    if (!load_joint_params_()) {
      return false;
    }
    if (!configure_motor_()) {
      return false;
    }
    if (!activate_motor_()) {
      return false;
    }
    log_parameters_();
    const bool ok = run_open_close_();
    // Always leave the motor torque-off, including on Ctrl+C abort.
    stop_motor_();
    return ok;
  }

 private:
  // -- setup ----------------------------------------------------------------

  bool init_transport_() {
    auto serial_port = std::make_unique<feetech_driver::SerialPort>(usb_port_);
    if (const auto result = serial_port->configure(); !result) {
      RCLCPP_ERROR(get_logger(), "Failed to configure serial port '%s': %s", usb_port_.c_str(),
                   result.error().c_str());
      return false;
    }
    protocol_ = std::make_unique<feetech_driver::CommunicationProtocol>(std::move(serial_port));
    return true;
  }

  bool load_joint_params_() {
    if (joint_config_file_.empty()) {
      RCLCPP_ERROR(get_logger(), "Parameter 'joint_config_file' is required");
      return false;
    }

    auto config = load_joint_config(joint_config_file_);
    if (!config) {
      RCLCPP_ERROR(get_logger(), "Failed to load joint config from '%s'", joint_config_file_.c_str());
      return false;
    }

    const auto it = config->find(joint_name_);
    if (it == config->end()) {
      RCLCPP_ERROR(get_logger(), "Joint '%s' not found in '%s'", joint_name_.c_str(), joint_config_file_.c_str());
      return false;
    }
    params_ = it->second;

    const auto id_it = params_.find("id");
    if (id_it == params_.end()) {
      RCLCPP_ERROR(get_logger(), "Joint '%s' has no 'id' parameter", joint_name_.c_str());
      return false;
    }
    id_ = static_cast<uint8_t>(std::stoi(id_it->second));

    // Resolve the effective max torque limit: the ROS param wins, otherwise
    // fall back to the YAML value, otherwise full torque. This single value is
    // used both to configure the EPROM max_torque_limit and as the dynamic
    // torque limit applied on every write_position.
    const char* torque_source = nullptr;
    if (max_torque_percent_ >= 0.0) {
      // Percentage (0..100 %) -> raw register units (0..1000, unit 0.1%).
      effective_max_torque_ = static_cast<int>(max_torque_percent_ * 10.0 + 0.5);
      torque_source = "ROS param 'max_torque' (%)";
    } else if (const auto it_torque = params_.find("max_torque_limit"); it_torque != params_.end()) {
      effective_max_torque_ = std::stoi(it_torque->second);
      torque_source = "YAML 'max_torque_limit' (raw)";
    } else {
      effective_max_torque_ = kMaxTorque;
      torque_source = "default (full torque)";
    }
    effective_max_torque_ = std::clamp(effective_max_torque_, 0, kMaxTorque);
    params_["max_torque_limit"] = std::to_string(effective_max_torque_);
    RCLCPP_INFO(get_logger(), "Effective max torque = %d raw (%.1f %%, source: %s)", effective_max_torque_,
                effective_max_torque_ / 10.0, torque_source);

    RCLCPP_INFO(get_logger(), "Controlling joint '%s' (id=%d) from '%s'", joint_name_.c_str(), id_,
                joint_config_file_.c_str());
    return true;
  }

  // Mirrors FeetechHardwareInterface::configure_joints_ for a single motor.
  bool configure_motor_() {
    if (const auto result = protocol_->ping(id_); !result) {
      RCLCPP_ERROR(get_logger(), "Ping to id=%d failed: %s", id_, result.error().c_str());
      return false;
    }

    // Disable torque and unlock EPROM before writing parameters.
    if (const auto result = protocol_->disable_torque(id_); !result) {
      RCLCPP_ERROR(get_logger(), "disable_torque -> %s", result.error().c_str());
      return false;
    }

    // Single-byte parameters (0-255).
    for (const auto& [name, address] : {std::pair{"p_coefficient", SMS_STS_P_COEF},
                                        {"d_coefficient", SMS_STS_D_COEF},
                                        {"i_coefficient", SMS_STS_I_COEF},
                                        {"overload_torque", SMS_STS_OVERLOAD_TORQUE},
                                        {"return_delay_time", SMS_STS_RETURN_DELAY},
                                        {"acceleration", SMS_STS_ACC}}) {
      if (!write_byte_param_(name, static_cast<uint8_t>(address))) {
        return false;
      }
    }

    // Two-byte unsigned parameters.
    for (const auto& [name, address] : {std::pair{"range_min", SMS_STS_MIN_ANGLE_LIMIT_L},
                                        {"range_max", SMS_STS_MAX_ANGLE_LIMIT_L},
                                        {"max_torque_limit", SMS_STS_MAX_TORQUE_L},
                                        {"protection_current", SMS_STS_PROTECTION_CURRENT_L}}) {
      if (!write_word_param_(name, static_cast<uint8_t>(address))) {
        return false;
      }
    }

    // Two-byte signed parameters (sign-magnitude encoding).
    if (const auto param_it = params_.find("homing_offset"); param_it != params_.end()) {
      std::array<uint8_t, 2> buf{};
      const int value = feetech_driver::encode_sign_magnitude(std::stoi(param_it->second),
                                                              SMS_STS_SIGN_BIT_HOMING_OFFSET);
      feetech_driver::to_sts(&buf[0], &buf[1], value);
      if (const auto result = protocol_->write(id_, SMS_STS_OFS_L, buf); !result) {
        RCLCPP_ERROR(get_logger(), "write homing_offset -> %s", result.error().c_str());
        return false;
      }
    }

    // Lock EPROM after writing parameters.
    if (const auto result = protocol_->lock_eprom(id_); !result) {
      RCLCPP_ERROR(get_logger(), "lock_eprom -> %s", result.error().c_str());
      return false;
    }

    // Position control mode for write_position.
    if (const auto result = protocol_->set_mode(id_, feetech_driver::OperationMode::kPosition); !result) {
      RCLCPP_ERROR(get_logger(), "set_mode(position) -> %s", result.error().c_str());
      return false;
    }

    RCLCPP_INFO(get_logger(), "Motor id=%d configured", id_);
    return true;
  }

  bool activate_motor_() {
    if (const auto result = protocol_->set_torque(id_, true); !result) {
      RCLCPP_ERROR(get_logger(), "set_torque(true) -> %s", result.error().c_str());
      return false;
    }
    RCLCPP_INFO(get_logger(), "Motor id=%d activated (torque enabled)", id_);
    return true;
  }

  void stop_motor_() {
    if (!protocol_) {
      return;
    }
    if (const auto result = protocol_->set_torque(id_, false); !result) {
      RCLCPP_WARN(get_logger(), "Failed to disable torque on exit: %s", result.error().c_str());
    } else {
      RCLCPP_INFO(get_logger(), "Motor id=%d torque disabled", id_);
    }
  }

  // -- read back / logging --------------------------------------------------

  void log_parameters_() {
    RCLCPP_INFO(get_logger(), "----- Read-back parameters for id=%d -----", id_);
    log_word_("max_torque_limit", SMS_STS_MAX_TORQUE_L);
    log_word_("min_angle_limit", SMS_STS_MIN_ANGLE_LIMIT_L);
    log_word_("max_angle_limit", SMS_STS_MAX_ANGLE_LIMIT_L);
    log_word_("protection_current", SMS_STS_PROTECTION_CURRENT_L);
    log_word_("p_coefficient", SMS_STS_P_COEF);
    log_word_("d_coefficient", SMS_STS_D_COEF);
    log_word_("i_coefficient", SMS_STS_I_COEF);

    // Commanded / effective limits in both metric and internal (servo) units.
    const int max_velocity_ticks = feetech_driver::from_radians(max_velocity_rad_s_);
    RCLCPP_INFO(get_logger(), "  max_velocity = %.3f rad/s (%d ticks/s internal)", max_velocity_rad_s_,
                max_velocity_ticks);

    if (const auto torque = protocol_->read_word(id_, SMS_STS_MAX_TORQUE_L); torque) {
      // The max_torque_limit register is 0..1000 = 0..100.0% of rated torque.
      RCLCPP_INFO(get_logger(), "  max_effort   = %.1f %% (%d / 1000 raw internal)", torque.value() / 10.0,
                  torque.value());
    } else {
      RCLCPP_WARN(get_logger(), "  failed to read max_effort: %s", torque.error().c_str());
    }

    if (const auto position = protocol_->read_position(id_); position) {
      RCLCPP_INFO(get_logger(), "  present_position = %d ticks", position.value());
    }
    RCLCPP_INFO(get_logger(), "------------------------------------------");
  }

  // -- motion ---------------------------------------------------------------

  // How a position command is sent to the servo.
  enum class WriteMode { kWritePosition, kSyncWritePosition, kRegWritePosition };

  static const char* write_mode_name_(WriteMode mode) {
    switch (mode) {
      case WriteMode::kWritePosition:
        return "write_position";
      case WriteMode::kSyncWritePosition:
        return "sync_write_position";
      case WriteMode::kRegWritePosition:
        return "reg_write_position";
    }
    return "unknown";
  }

  bool run_open_close_() {
    RCLCPP_INFO(get_logger(), "=== Cycle 1/3: write_position (single-servo command) ===");
    if (!run_one_cycle_(WriteMode::kWritePosition)) {
      return false;
    }

    RCLCPP_INFO(get_logger(), "waiting %.1f seconds before next cycle", cycle_wait_seconds_);
    if (!sleep_(cycle_wait_seconds_)) {
      return false;
    }

    RCLCPP_INFO(get_logger(), "=== Cycle 2/3: sync_write_position (sync command) ===");
    if (!run_one_cycle_(WriteMode::kSyncWritePosition)) {
      return false;
    }

    RCLCPP_INFO(get_logger(), "waiting %.1f seconds before next cycle", cycle_wait_seconds_);
    if (!sleep_(cycle_wait_seconds_)) {
      return false;
    }

    RCLCPP_INFO(get_logger(), "=== Cycle 3/3: reg_write_position + action (staged command) ===");
    return run_one_cycle_(WriteMode::kRegWritePosition);
  }

  bool run_one_cycle_(WriteMode mode) {
    const char* method = write_mode_name_(mode);

    RCLCPP_INFO(get_logger(), "[%s] -> open (%.3f rad)", method, open_position_rad_);
    if (!move_to_(open_position_rad_, mode)) {
      return false;
    }
    if (!sleep_(wait_seconds_)) {
      return false;
    }

    RCLCPP_INFO(get_logger(), "[%s] -> close (%.3f rad)", method, close_position_rad_);
    if (!move_to_(close_position_rad_, mode)) {
      return false;
    }
    return true;
  }

  bool move_to_(double position_rad, WriteMode mode) {
    // Radians -> servo ticks, centered on the midpoint (0 rad = 2048 ticks).
    int position_ticks = feetech_driver::from_radians(position_rad) + feetech_driver::kStsMidpoint;

    // Valid servo range is [0, resolution-1]. Clamp (instead of letting
    // encode_sign_magnitude throw) so a bad parameter can't crash the node.
    constexpr int kMaxTick = static_cast<int>(feetech_driver::kStsResolution) - 1;
    if (position_ticks < 0 || position_ticks > kMaxTick) {
      RCLCPP_WARN(get_logger(), "position %.3f rad = %d ticks is out of range [0, %d], clamping", position_rad,
                  position_ticks, kMaxTick);
      position_ticks = std::clamp(position_ticks, 0, kMaxTick);
    }

    // Apply velocity and torque limit dynamically on every command.
    const int speed_ticks = feetech_driver::from_radians(max_velocity_rad_s_);

    feetech_driver::Result result;
    switch (mode) {
      case WriteMode::kWritePosition:
        result = protocol_->write_position(id_, position_ticks, speed_ticks, acceleration_, effective_max_torque_);
        break;
      case WriteMode::kSyncWritePosition:
        result = protocol_->sync_write_position(std::vector<uint8_t>{id_}, {position_ticks}, {speed_ticks},
                                                {acceleration_}, {effective_max_torque_});
        break;
      case WriteMode::kRegWritePosition:
        // reg_write stages the command; reg_write_action triggers execution.
        result = protocol_->reg_write_position(id_, position_ticks, speed_ticks, acceleration_, effective_max_torque_)
                     .and_then([&] { return protocol_->reg_write_action(id_); });
        break;
    }
    if (!result) {
      RCLCPP_ERROR(get_logger(), "move to %.3f rad (%d ticks, %d ticks/s, torque %d) -> %s", position_rad,
                   position_ticks, speed_ticks, effective_max_torque_, result.error().c_str());
      return false;
    }
    return true;
  }

  // -- helpers --------------------------------------------------------------

  bool write_byte_param_(const std::string& name, uint8_t address) {
    const auto it = params_.find(name);
    if (it == params_.end()) {
      return true;  // not specified -> leave motor default
    }
    const auto value = static_cast<uint8_t>(std::stoi(it->second));
    if (const auto result = protocol_->write(id_, address, std::experimental::make_array(value)); !result) {
      RCLCPP_ERROR(get_logger(), "write '%s'=%s -> %s", name.c_str(), it->second.c_str(), result.error().c_str());
      return false;
    }
    RCLCPP_INFO(get_logger(), "  wrote %s = %s", name.c_str(), it->second.c_str());
    return true;
  }

  bool write_word_param_(const std::string& name, uint8_t address) {
    const auto it = params_.find(name);
    if (it == params_.end()) {
      return true;
    }
    std::array<uint8_t, 2> buf{};
    feetech_driver::to_sts(&buf[0], &buf[1], std::stoi(it->second));
    if (const auto result = protocol_->write(id_, address, buf); !result) {
      RCLCPP_ERROR(get_logger(), "write '%s'=%s -> %s", name.c_str(), it->second.c_str(), result.error().c_str());
      return false;
    }
    RCLCPP_INFO(get_logger(), "  wrote %s = %s", name.c_str(), it->second.c_str());
    return true;
  }

  void log_word_(const std::string& name, uint8_t address) {
    if (const auto value = protocol_->read_word(id_, address); value) {
      RCLCPP_INFO(get_logger(), "  %s = %d", name.c_str(), value.value());
    } else {
      RCLCPP_WARN(get_logger(), "  failed to read %s: %s", name.c_str(), value.error().c_str());
    }
  }

  // Interruptible sleep: returns false as soon as a shutdown (e.g. Ctrl+C) is
  // requested, so the caller can abort the sequence promptly.
  bool sleep_(double seconds) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(50ms);
    }
    return rclcpp::ok();
  }

  // Parameters
  std::string usb_port_;
  std::string joint_config_file_;
  std::string joint_name_;
  double open_position_rad_{};
  double close_position_rad_{};
  double max_velocity_rad_s_{};
  double max_torque_percent_{};
  int acceleration_{};
  double wait_seconds_{};
  double cycle_wait_seconds_{};

  // Full-scale torque limit (max_torque_limit register range 0..1000).
  static constexpr int kMaxTorque = 1000;

  // Runtime
  std::unique_ptr<feetech_driver::CommunicationProtocol> protocol_;
  JointParams params_;
  uint8_t id_{0};
  int effective_max_torque_{kMaxTorque};
};

}  // namespace feetech_ros2_driver

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<feetech_ros2_driver::SingleMotorNode>();
  const bool ok = node->run();
  rclcpp::shutdown();
  return ok ? EXIT_SUCCESS : EXIT_FAILURE;
}
