#ifndef IDS830_HW__IDS830_HW_HPP_
#define IDS830_HW__IDS830_HW_HPP_

#include <string>
#include <vector>
#include <array>
#include <mutex>
#include <thread>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <unordered_map>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace ids830_hw
{

/**
 * @brief ros2_control hardware interface for IDS830 Low-Voltage DC Servo.
 *
 * Communication: CAN bus via SLCAN (Serial Line CAN) adapter.
 * Same SLCAN approach as kinco_canopen_hw in tbm_ws.
 *
 * IDS830 CAN Protocol (from manual):
 *   - Standard Frame (11-bit ID), 8-byte payload
 *   - Payload: [Group][FuncCode][Reg1][Data1_H][Data1_L][Reg2][Data2_H][Data2_L]
 *   - Function codes: 0x1A (Write), 0x2A (Read), 0x2B (Read Response)
 *   - Speed formula: value = (RPM / 3000) * 8192
 *
 * Key registers:
 *   - 0x00: Motor Enable (0x0001=ON, 0x0000=OFF)
 *   - 0x06: Target Speed
 *   - 0xE8: Encoder High 16-bit (read)
 *   - 0xE9: Encoder Low 16-bit (read)
 */
class IDS830HW : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(IDS830HW)

  // ─── Lifecycle ───────────────────────────────────────────────
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  // ─── Interfaces ──────────────────────────────────────────────
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // ─── Read / Write ────────────────────────────────────────────
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // ─── Serial SLCAN helpers ────────────────────────────────────
  bool serial_open(const std::string & port, int baudrate);
  void serial_close();
  void serial_write_str(const std::string & s);
  std::string serial_read_line(int timeout_ms);

  // ─── SLCAN protocol ──────────────────────────────────────────
  void slcan_open(int can_speed_code);
  void slcan_close();
  void slcan_send(uint32_t id, const uint8_t * data, uint8_t len);
  std::string slcan_format(uint32_t id, const uint8_t * data, uint8_t len);
  bool slcan_recv(uint32_t & id, uint8_t * data, uint8_t & len, int timeout_ms);

  // ─── RX background thread ───────────────────────────────────
  void rx_loop();
  std::thread rx_thread_;
  std::atomic<bool> rx_running_{false};
  std::mutex rx_mutex_;
  std::unordered_map<uint32_t, std::vector<uint8_t>> rx_responses_;
  std::unordered_map<uint32_t, std::chrono::steady_clock::time_point> rx_response_stamps_;

  // ─── IDS830 CAN commands ─────────────────────────────────────
  /** Unlock PC control mode (clear bit 5 of register 0x36) */
  bool motor_unlock_pc_mode(uint8_t can_id);

  /** Set speed mode: PC Digital Input (register 0x02 = 0xC4) */
  void motor_set_speed_mode(uint8_t can_id);

  /** Enable motor (register 0x00 = 0x0001) */
  void motor_enable(uint8_t can_id);

  /** Disable motor (register 0x00 = 0x0000) */
  void motor_disable(uint8_t can_id);

  /**
   * Set motor speed + keep enabled (combined single-frame command).
   * Writes Reg 0x06 (speed) + Reg 0x00 (enable=0x0001) in one CAN frame.
   * @param can_id   CAN arbitration ID of the motor
   * @param rpm      Target RPM (negative = reverse)
   */
  void motor_set_speed(uint8_t can_id, int16_t rpm);

  /**
   * Request encoder reading from motor.
   * Reads Reg 0xE8 (encoder high) + Reg 0xE9 (encoder low).
   */
  void motor_request_encoder(uint8_t can_id);

  /**
   * Parse encoder response from RX buffer.
   * @param can_id   CAN ID to look for
   * @param out_pulses  Output: accumulated encoder pulses (signed 32-bit)
   * @return true if fresh response found
   */
  bool motor_parse_encoder(uint8_t can_id, int32_t & out_pulses);

  // ─── Unit conversion ────────────────────────────────────────
  /** Convert wheel rad/s to motor RPM (through gear ratio) */
  int16_t wheel_rads_to_motor_rpm(double rad_per_sec) const;

  /** Convert motor RPM to wheel rad/s (through gear ratio) */
  double motor_rpm_to_wheel_rads(int16_t rpm) const;

  /** Convert RPM to IDS830 internal speed value: (RPM / 3000) * 8192 */
  int16_t rpm_to_ids_value(int16_t rpm) const;

  /** Convert IDS830 internal speed value to RPM */
  int16_t ids_value_to_rpm(int16_t value) const;

  // ─── Configuration ──────────────────────────────────────────
  std::string can_channel_;           // Serial port for SLCAN adapter
  int serial_baud_ = 921600;          // Serial baudrate to SLCAN adapter
  uint8_t left_id_ = 1;              // CAN ID for left motor
  uint8_t right_id_ = 2;             // CAN ID for right motor
  int gear_ratio_ = 9;               // Motor-to-wheel gear ratio
  int encoder_ppr_ = 4096;           // Encoder pulses per motor revolution
  double wheel_radius_m_ = 0.081;    // Wheel radius in meters
  double wheel_track_m_ = 0.39;      // Distance between wheels
  int can_speed_code_ = 6;           // SLCAN speed: S6=500kbps

  // ─── Serial ─────────────────────────────────────────────────
  int serial_fd_ = -1;

  // ─── State ──────────────────────────────────────────────────
  std::array<double, 2> hw_commands_ = {0.0, 0.0};     // velocity commands (rad/s)
  std::array<double, 2> hw_positions_ = {0.0, 0.0};    // wheel positions (rad)
  std::array<double, 2> hw_velocities_ = {0.0, 0.0};   // wheel velocities (rad/s)

  // Encoder tracking
  std::array<int32_t, 2> prev_encoder_ = {0, 0};
  std::array<bool, 2> encoder_initialized_ = {false, false};

  // Idle management
  bool motors_idle_ = true;
  int settle_count_ = 0;
  static constexpr int SETTLE_THRESHOLD = 50;  // ~0.5s at 100Hz

  // ─── IDS830 protocol constants ──────────────────────────────
  static constexpr uint8_t GROUP_DEFAULT    = 0x00;
  static constexpr uint8_t FUNC_WRITE       = 0x1A;
  static constexpr uint8_t FUNC_READ        = 0x2A;
  static constexpr uint8_t FUNC_READ_RESP   = 0x2B;
  static constexpr uint8_t REG_ENABLE       = 0x00;
  static constexpr uint8_t REG_SPEED        = 0x06;
  static constexpr uint8_t REG_ENCODER_HIGH = 0xE8;
  static constexpr uint8_t REG_ENCODER_LOW  = 0xE9;
  static constexpr uint8_t REG_PC_PLC_CTRL  = 0x36;  // PC/PLC mode register
  static constexpr uint8_t REG_SPEED_MODE   = 0x02;  // Speed mode register
  static constexpr uint8_t REG_EMPTY        = 0xFF;
  static constexpr int16_t MAX_RPM          = 3000;
  static constexpr int16_t SPEED_SCALE      = 8192;
};

}  // namespace ids830_hw

#endif  // IDS830_HW__IDS830_HW_HPP_
