/**
 * @file ids830_hw.cpp
 * @brief ros2_control hardware interface for IDS830 Low-Voltage DC Servo.
 *
 * Communication: CAN bus via SLCAN (Serial Line CAN) adapter.
 * Protocol: IDS830 proprietary CAN protocol (NOT CANopen).
 *
 * IDS830 CAN Frame (8 bytes):
 *   [Group][FuncCode][Reg1][Data1_H][Data1_L][Reg2][Data2_H][Data2_L]
 *
 * Speed Formula: value = (RPM / 3000) * 8192
 *
 * Based on kinco_canopen_hw SLCAN architecture from tbm_ws.
 */

#include "ids830_hw/ids830_hw.hpp"

#include <cstring>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <algorithm>

// Linux serial
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/select.h>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace ids830_hw
{

static const auto LOG = "IDS830HW";

// ============================================================
// Serial helpers (from kinco_canopen_hw)
// ============================================================

bool IDS830HW::serial_open(const std::string & port, int baudrate)
{
  serial_fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (serial_fd_ < 0) {
    RCLCPP_ERROR(rclcpp::get_logger(LOG),
      "Cannot open serial port '%s': %s", port.c_str(), strerror(errno));
    return false;
  }

  struct termios tty;
  std::memset(&tty, 0, sizeof(tty));
  if (tcgetattr(serial_fd_, &tty) != 0) {
    RCLCPP_ERROR(rclcpp::get_logger(LOG), "tcgetattr failed: %s", strerror(errno));
    ::close(serial_fd_);
    serial_fd_ = -1;
    return false;
  }

  speed_t speed;
  switch (baudrate) {
    case 9600:   speed = B9600;   break;
    case 19200:  speed = B19200;  break;
    case 38400:  speed = B38400;  break;
    case 57600:  speed = B57600;  break;
    case 115200: speed = B115200; break;
    case 230400: speed = B230400; break;
    case 460800: speed = B460800; break;
    case 921600: speed = B921600; break;
    default:     speed = B921600; break;
  }

  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);

  // 8N1, no flow control
  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS);
  tty.c_cflag |= CLOCAL | CREAD;

  // Raw mode
  tty.c_iflag &= ~(IXON | IXOFF | IXANY | IGNBRK | BRKINT | PARMRK |
                    ISTRIP | INLCR | IGNCR | ICRNL);
  tty.c_oflag &= ~OPOST;
  tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);

  tty.c_cc[VMIN]  = 0;
  tty.c_cc[VTIME] = 1;

  if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
    RCLCPP_ERROR(rclcpp::get_logger(LOG), "tcsetattr failed: %s", strerror(errno));
    ::close(serial_fd_);
    serial_fd_ = -1;
    return false;
  }

  tcflush(serial_fd_, TCIOFLUSH);

  RCLCPP_INFO(rclcpp::get_logger(LOG),
    "Serial port %s opened at %d baud", port.c_str(), baudrate);
  return true;
}

void IDS830HW::serial_close()
{
  if (serial_fd_ >= 0) {
    ::close(serial_fd_);
    serial_fd_ = -1;
  }
}

void IDS830HW::serial_write_str(const std::string & s)
{
  if (serial_fd_ < 0) return;
  ::write(serial_fd_, s.c_str(), s.size());
}

std::string IDS830HW::serial_read_line(int timeout_ms)
{
  if (serial_fd_ < 0) return "";

  std::string line;
  auto deadline = std::chrono::steady_clock::now() +
                  std::chrono::milliseconds(timeout_ms);

  while (std::chrono::steady_clock::now() < deadline) {
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(serial_fd_, &fds);
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 5000;

    if (select(serial_fd_ + 1, &fds, nullptr, nullptr, &tv) > 0) {
      char c;
      if (::read(serial_fd_, &c, 1) == 1) {
        if (c == '\r' || c == '\n') {
          if (!line.empty()) return line;
        } else {
          line += c;
        }
      }
    }
  }
  return line;
}

// ============================================================
// SLCAN protocol (from kinco_canopen_hw)
// ============================================================

void IDS830HW::slcan_open(int can_speed_code)
{
  serial_write_str("C\r");
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  tcflush(serial_fd_, TCIOFLUSH);

  // S0=10k S1=20k S2=50k S3=100k S4=125k S5=250k S6=500k S7=800k S8=1M
  char cmd[4];
  std::snprintf(cmd, sizeof(cmd), "S%d\r", can_speed_code);
  serial_write_str(cmd);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  serial_write_str("O\r");
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  RCLCPP_INFO(rclcpp::get_logger(LOG), "SLCAN channel opened (speed code S%d)", can_speed_code);
}

void IDS830HW::slcan_close()
{
  serial_write_str("C\r");
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
}

void IDS830HW::slcan_send(uint32_t id, const uint8_t * data, uint8_t len)
{
  serial_write_str(slcan_format(id, data, len));
}

std::string IDS830HW::slcan_format(uint32_t id, const uint8_t * data, uint8_t len)
{
  // SLCAN format: tIIILDD..DD\r  (t=standard frame, III=ID 3 hex digits, L=length)
  char buf[32];
  int pos = std::snprintf(buf, sizeof(buf), "t%03X%d", id & 0x7FF, len);
  for (int i = 0; i < len; ++i) {
    pos += std::snprintf(buf + pos, sizeof(buf) - pos, "%02X", data[i]);
  }
  buf[pos++] = '\r';
  return std::string(buf, pos);
}

bool IDS830HW::slcan_recv(uint32_t & id, uint8_t * data, uint8_t & len, int timeout_ms)
{
  std::string line = serial_read_line(timeout_ms);
  if (line.empty() || line[0] != 't') return false;
  if (line.size() < 5) return false;

  char id_str[4] = {line[1], line[2], line[3], '\0'};
  id = static_cast<uint32_t>(std::strtoul(id_str, nullptr, 16));
  len = static_cast<uint8_t>(line[4] - '0');

  if (len > 8) return false;
  if (line.size() < 5 + len * 2) return false;

  for (int i = 0; i < len; ++i) {
    char byte_str[3] = {line[5 + i * 2], line[6 + i * 2], '\0'};
    data[i] = static_cast<uint8_t>(std::strtoul(byte_str, nullptr, 16));
  }
  return true;
}

// ============================================================
// RX background thread
// ============================================================

void IDS830HW::rx_loop()
{
  while (rx_running_) {
    uint32_t id;
    uint8_t data[8];
    uint8_t len;

    if (slcan_recv(id, data, len, 20)) {
      std::lock_guard<std::mutex> lock(rx_mutex_);
      rx_responses_[id] = std::vector<uint8_t>(data, data + len);
      rx_response_stamps_[id] = std::chrono::steady_clock::now();
    }
  }
}

// ============================================================
// IDS830 Motor Commands
// ============================================================

void IDS830HW::motor_enable(uint8_t can_id)
{
  // Payload: [0x00, 0x1A, 0x00, 0x00, 0x01, 0xFF, 0x00, 0x00]
  // Group=0, Write, Reg=0x00(Enable), Data=0x0001, Reg2=empty(0xFF)
  uint8_t data[8] = {
    GROUP_DEFAULT,
    FUNC_WRITE,
    REG_ENABLE,
    0x00, 0x01,       // Enable = 0x0001
    REG_EMPTY,
    0x00, 0x00
  };
  slcan_send(can_id, data, 8);
  RCLCPP_INFO(rclcpp::get_logger(LOG), "Motor CAN_ID=0x%02X: ENABLED", can_id);
}

void IDS830HW::motor_disable(uint8_t can_id)
{
  // Payload: [0x00, 0x1A, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x00]
  uint8_t data[8] = {
    GROUP_DEFAULT,
    FUNC_WRITE,
    REG_ENABLE,
    0x00, 0x00,       // Disable = 0x0000
    REG_EMPTY,
    0x00, 0x00
  };
  slcan_send(can_id, data, 8);
  RCLCPP_INFO(rclcpp::get_logger(LOG), "Motor CAN_ID=0x%02X: DISABLED", can_id);
}

bool IDS830HW::motor_unlock_pc_mode(uint8_t can_id)
{
  // Step 1: Read current value of register 0x36
  uint8_t read_cmd[8] = {
    GROUP_DEFAULT, FUNC_READ,
    REG_PC_PLC_CTRL, 0x00, 0x00,
    REG_EMPTY, 0x00, 0x00
  };
  slcan_send(can_id, read_cmd, 8);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Step 2: Parse the response
  int32_t current_val = 0;
  {
    std::lock_guard<std::mutex> lock(rx_mutex_);
    auto it = rx_responses_.find(can_id);
    if (it != rx_responses_.end() && it->second.size() >= 8 &&
        it->second[1] == FUNC_READ_RESP) {
      current_val = (it->second[3] << 8) | it->second[4];
      rx_responses_.erase(it);
    } else {
      RCLCPP_WARN(rclcpp::get_logger(LOG),
        "Motor CAN_ID=0x%02X: No response for reg 0x36 read", can_id);
      return false;
    }
  }

  // Step 3: Clear bit 5 (PLC mode bit) and write back
  int32_t new_val = current_val & ~(1 << 5);
  uint8_t write_cmd[8] = {
    GROUP_DEFAULT, FUNC_WRITE,
    REG_PC_PLC_CTRL,
    static_cast<uint8_t>((new_val >> 8) & 0xFF),
    static_cast<uint8_t>(new_val & 0xFF),
    REG_EMPTY, 0x00, 0x00
  };
  slcan_send(can_id, write_cmd, 8);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  RCLCPP_INFO(rclcpp::get_logger(LOG),
    "Motor CAN_ID=0x%02X: PC mode UNLOCKED (0x36: 0x%04X -> 0x%04X)",
    can_id, current_val, new_val);
  return true;
}

void IDS830HW::motor_set_speed_mode(uint8_t can_id)
{
  // Register 0x02 = 0xC4 (Speed Mode, PC Digital Input)
  uint8_t data[8] = {
    GROUP_DEFAULT, FUNC_WRITE,
    REG_SPEED_MODE,
    0x00, 0xC4,
    REG_EMPTY, 0x00, 0x00
  };
  slcan_send(can_id, data, 8);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  RCLCPP_INFO(rclcpp::get_logger(LOG),
    "Motor CAN_ID=0x%02X: Speed Mode set (PC Digital)", can_id);
}

void IDS830HW::motor_set_speed(uint8_t can_id, int16_t rpm)
{
  // Convert RPM to IDS830 internal value
  int16_t speed_val = rpm_to_ids_value(rpm);

  // Handle negative (two's complement 16-bit)
  uint16_t uval = static_cast<uint16_t>(speed_val);
  uint8_t high_byte = (uval >> 8) & 0xFF;
  uint8_t low_byte  = uval & 0xFF;

  // Combined command: Set speed (Reg 0x06) + Enable (Reg 0x00 = 0x0001)
  // Payload: [0x00, 0x1A, 0x06, High_RPM, Low_RPM, 0x00, 0x00, 0x01]
  uint8_t data[8] = {
    GROUP_DEFAULT,
    FUNC_WRITE,
    REG_SPEED,
    high_byte, low_byte,
    REG_ENABLE,
    0x00, 0x01          // Keep enabled
  };
  slcan_send(can_id, data, 8);
}

void IDS830HW::motor_request_encoder(uint8_t can_id)
{
  // Read request: Reg 0xE8 (encoder high) + Reg 0xE9 (encoder low)
  // Payload: [0x00, 0x2A, 0xE8, 0x00, 0x00, 0xE9, 0x00, 0x00]
  uint8_t data[8] = {
    GROUP_DEFAULT,
    FUNC_READ,
    REG_ENCODER_HIGH,
    0x00, 0x00,
    REG_ENCODER_LOW,
    0x00, 0x00
  };
  slcan_send(can_id, data, 8);
}

bool IDS830HW::motor_parse_encoder(uint8_t can_id, int32_t & out_pulses)
{
  std::lock_guard<std::mutex> lock(rx_mutex_);

  auto it = rx_responses_.find(can_id);
  auto ts = rx_response_stamps_.find(can_id);
  if (it == rx_responses_.end() || ts == rx_response_stamps_.end()) {
    return false;
  }

  // Check freshness (100ms timeout)
  auto age = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - ts->second).count();
  if (age > 100) {
    return false;
  }

  const auto & resp = it->second;
  if (resp.size() < 8) return false;

  // Verify it's a read response (function code 0x2B)
  if (resp[1] != FUNC_READ_RESP) return false;

  // Verify registers 0xE8 and 0xE9
  if (resp[2] != REG_ENCODER_HIGH || resp[5] != REG_ENCODER_LOW) return false;

  // Combine 4 bytes into signed 32-bit encoder value
  uint16_t enc_high = (static_cast<uint16_t>(resp[3]) << 8) | resp[4];
  uint16_t enc_low  = (static_cast<uint16_t>(resp[6]) << 8) | resp[7];
  uint32_t raw = (static_cast<uint32_t>(enc_high) << 16) | enc_low;
  out_pulses = static_cast<int32_t>(raw);

  // Clear used response
  rx_responses_.erase(it);

  return true;
}

// ============================================================
// Unit Conversion
// ============================================================

int16_t IDS830HW::rpm_to_ids_value(int16_t rpm) const
{
  // IDS830 formula: value = (RPM / 3000) * 8192
  return static_cast<int16_t>((static_cast<int32_t>(rpm) * SPEED_SCALE) / MAX_RPM);
}

int16_t IDS830HW::ids_value_to_rpm(int16_t value) const
{
  // Inverse: RPM = (value / 8192) * 3000
  return static_cast<int16_t>((static_cast<int32_t>(value) * MAX_RPM) / SPEED_SCALE);
}

int16_t IDS830HW::wheel_rads_to_motor_rpm(double rad_per_sec) const
{
  double wheel_rpm = rad_per_sec * 60.0 / (2.0 * M_PI);
  double motor_rpm = wheel_rpm * gear_ratio_;
  return static_cast<int16_t>(std::clamp(motor_rpm, -3000.0, 3000.0));
}

double IDS830HW::motor_rpm_to_wheel_rads(int16_t rpm) const
{
  double wheel_rpm = static_cast<double>(rpm) / gear_ratio_;
  return wheel_rpm * 2.0 * M_PI / 60.0;
}

// ============================================================
// Lifecycle
// ============================================================

hardware_interface::CallbackReturn IDS830HW::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  auto get = [&](const std::string & key, const std::string & def) -> std::string {
    auto it = info.hardware_parameters.find(key);
    return (it != info.hardware_parameters.end()) ? it->second : def;
  };

  can_channel_      = get("can_channel",       "/dev/ttyACM0");
  serial_baud_      = std::stoi(get("serial_baudrate", "921600"));
  left_id_          = static_cast<uint8_t>(std::stoi(get("left_motor_id",  "1")));
  right_id_         = static_cast<uint8_t>(std::stoi(get("right_motor_id", "2")));
  gear_ratio_       = std::stoi(get("gear_ratio",       "9"));
  encoder_ppr_      = std::stoi(get("encoder_ppr",      "4096"));
  wheel_radius_m_   = std::stod(get("wheel_radius_m",   "0.081"));
  wheel_track_m_    = std::stod(get("wheel_track_m",    "0.39"));
  can_speed_code_   = std::stoi(get("can_speed_code",   "6"));  // S6=500kbps

  if (info.joints.size() != 2) {
    RCLCPP_ERROR(rclcpp::get_logger(LOG),
      "Expected 2 joints, got %zu", info.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger(LOG),
    "Init OK: channel=%s baud=%d left_id=0x%02X right_id=0x%02X gear=%d encoder_ppr=%d can_speed=S%d",
    can_channel_.c_str(), serial_baud_, left_id_, right_id_,
    gear_ratio_, encoder_ppr_, can_speed_code_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn IDS830HW::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger(LOG), "Configuring SLCAN for IDS830...");

  // Open serial port to SLCAN adapter
  if (!serial_open(can_channel_, serial_baud_)) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Open SLCAN channel
  slcan_open(can_speed_code_);

  // Start RX thread
  rx_running_ = true;
  rx_thread_ = std::thread(&IDS830HW::rx_loop, this);

  // Small delay for CAN bus to settle
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  // Unlock PC mode (critical! discovered via reverse engineering)
  RCLCPP_INFO(rclcpp::get_logger(LOG), "Unlocking PC mode for both motors...");
  motor_unlock_pc_mode(left_id_);
  motor_unlock_pc_mode(right_id_);

  // Set speed mode
  motor_set_speed_mode(left_id_);
  motor_set_speed_mode(right_id_);

  // Enable both motors
  motor_enable(left_id_);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  motor_enable(right_id_);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  RCLCPP_INFO(rclcpp::get_logger(LOG), "Both IDS830 motors configured and enabled");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn IDS830HW::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger(LOG), "Activating — ready for velocity commands");
  hw_commands_ = {0.0, 0.0};
  hw_positions_ = {0.0, 0.0};
  hw_velocities_ = {0.0, 0.0};
  prev_encoder_ = {0, 0};
  encoder_initialized_ = {false, false};
  motors_idle_ = true;
  settle_count_ = 0;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn IDS830HW::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger(LOG), "Deactivating — stopping motors");

  // Send zero speed to both motors
  motor_set_speed(left_id_, 0);
  motor_set_speed(right_id_, 0);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  // Disable motors
  motor_disable(left_id_);
  motor_disable(right_id_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn IDS830HW::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger(LOG), "Cleaning up");
  rx_running_ = false;
  if (rx_thread_.joinable()) rx_thread_.join();
  slcan_close();
  serial_close();
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ============================================================
// State / Command Interfaces
// ============================================================

std::vector<hardware_interface::StateInterface>
IDS830HW::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> si;
  for (size_t i = 0; i < 2; ++i) {
    si.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
    si.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
  }
  return si;
}

std::vector<hardware_interface::CommandInterface>
IDS830HW::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> ci;
  for (size_t i = 0; i < 2; ++i) {
    ci.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]);
  }
  return ci;
}

// ============================================================
// Read — Read encoder feedback from motors
// ============================================================

hardware_interface::return_type IDS830HW::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  const double dt = std::max(period.seconds(), 1e-4);

  // Request encoder data from both motors
  motor_request_encoder(left_id_);
  motor_request_encoder(right_id_);

  // Small delay for response
  std::this_thread::sleep_for(std::chrono::milliseconds(2));

  // Parse encoder responses
  int32_t enc_left = 0, enc_right = 0;
  bool left_ok = motor_parse_encoder(left_id_, enc_left);
  bool right_ok = motor_parse_encoder(right_id_, enc_right);

  // --- Left motor ---
  if (left_ok) {
    if (!encoder_initialized_[0]) {
      prev_encoder_[0] = enc_left;
      encoder_initialized_[0] = true;
    }
    int32_t delta = enc_left - prev_encoder_[0];
    prev_encoder_[0] = enc_left;

    // Convert encoder pulses to wheel radians
    // pulses_per_wheel_rev = encoder_ppr * gear_ratio
    double rad_delta = (static_cast<double>(delta) / (encoder_ppr_ * gear_ratio_)) * 2.0 * M_PI;
    hw_positions_[0] += rad_delta;
    hw_velocities_[0] = rad_delta / dt;
  }

  // --- Right motor (reversed direction) ---
  if (right_ok) {
    if (!encoder_initialized_[1]) {
      prev_encoder_[1] = enc_right;
      encoder_initialized_[1] = true;
    }
    int32_t delta = enc_right - prev_encoder_[1];
    prev_encoder_[1] = enc_right;

    double rad_delta = -(static_cast<double>(delta) / (encoder_ppr_ * gear_ratio_)) * 2.0 * M_PI;
    hw_positions_[1] += rad_delta;
    hw_velocities_[1] = rad_delta / dt;
  }

  // Debug log at ~1Hz
  static int dbg_count = 0;
  if (++dbg_count >= 100) {
    dbg_count = 0;
    if (std::abs(hw_velocities_[0]) > 0.01 || std::abs(hw_velocities_[1]) > 0.01) {
      RCLCPP_INFO(rclcpp::get_logger(LOG),
        "VEL L=%.3f R=%.3f rad/s | ENC L=%d R=%d | fresh L=%d R=%d",
        hw_velocities_[0], hw_velocities_[1],
        enc_left, enc_right,
        left_ok ? 1 : 0, right_ok ? 1 : 0);
    }
  }

  return hardware_interface::return_type::OK;
}

// ============================================================
// Write — Send velocity commands to motors
// ============================================================

hardware_interface::return_type IDS830HW::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Convert wheel rad/s to motor RPM
  int16_t rpm_left  = wheel_rads_to_motor_rpm(hw_commands_[0]);
  int16_t rpm_right = static_cast<int16_t>(-wheel_rads_to_motor_rpm(hw_commands_[1]));  // Right reversed

  // Small deadband
  constexpr int16_t DEADBAND = 3;
  if (std::abs(rpm_left)  < DEADBAND) rpm_left  = 0;
  if (std::abs(rpm_right) < DEADBAND) rpm_right = 0;

  bool both_zero = (rpm_left == 0 && rpm_right == 0);

  // ---- Idle management (from kinco_hw pattern) ----
  if (motors_idle_) {
    if (!both_zero) {
      motors_idle_ = false;
      settle_count_ = 0;
      RCLCPP_INFO(rclcpp::get_logger(LOG), "Motors waking up from idle");
    } else {
      return hardware_interface::return_type::OK;  // Skip sending when idle
    }
  }

  if (both_zero) {
    settle_count_++;
    if (settle_count_ <= SETTLE_THRESHOLD) {
      // Keep sending zero for a while to ensure motor stops
      motor_set_speed(left_id_, 0);
      motor_set_speed(right_id_, 0);
    }
    if (settle_count_ >= SETTLE_THRESHOLD && !motors_idle_) {
      motors_idle_ = true;
      RCLCPP_INFO(rclcpp::get_logger(LOG), "Both motors idle — CAN traffic stopped");
    }
  } else {
    settle_count_ = 0;
    // Send speed commands (combined with enable in single frame)
    motor_set_speed(left_id_, rpm_left);
    motor_set_speed(right_id_, rpm_right);
  }

  return hardware_interface::return_type::OK;
}

}  // namespace ids830_hw

// Register the plugin
PLUGINLIB_EXPORT_CLASS(ids830_hw::IDS830HW, hardware_interface::SystemInterface)
