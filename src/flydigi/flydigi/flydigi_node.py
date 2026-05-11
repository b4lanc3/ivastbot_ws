#!/usr/bin/env python3
"""
Flydigi Controller ROS2 Node - Differential Drive
Điều khiển robot 2 bánh vi sai (Kinco iWMC) qua /cmd_vel topic
Tương thích với các tay cầm Flydigi (Apex, Vader, Direwolf, v.v.)

Flydigi trên Linux thường nhận diện qua xinput mode.
Node sẽ tự detect button mapping dựa trên tên tay cầm.

Mapping mặc định (Xinput / Flydigi):
    Left Stick Y    -> Tiến/Lùi (linear.x)
    Right Stick X   -> Quay trái/phải (angular.z)
    Y (axis 3)      -> Tiến
    A (axis 0)      -> Lùi
    X (axis 2)      -> Quay trái
    B (axis 1)      -> Quay phải
    LB (L1)         -> Giảm tốc
    RB (R1)         -> Tăng tốc
    START           -> Thoát (smooth deceleration)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import pygame

# ─────────────────────────────────────────────────────────────
# Flydigi button mapping (Xinput mode - mặc định trên Linux)
# Flydigi thường nhận diện như: "Flydigi Vader 2" hoặc tương tự
# Xinput layout tương tự Xbox 360
# ─────────────────────────────────────────────────────────────
BTN_A  = 0    # Nút A (phía dưới)
BTN_B  = 1    # Nút B (phía phải)
BTN_X  = 2    # Nút X (phía trái)
BTN_Y  = 3    # Nút Y (phía trên)
BTN_LB = 4    # L1 / LB
BTN_RB = 5    # R1 / RB
BTN_BACK  = 6   # Back / Select
BTN_START = 7   # Start / Menu
BTN_HOME  = 8   # Home / Logo (nếu có)
BTN_L3 = 9      # Left stick press
BTN_R3 = 10     # Right stick press

# Axis mapping (Flydigi Xinput)
AXIS_LX = 0   # Left stick X
AXIS_LY = 1   # Left stick Y
AXIS_RX = 3   # Right stick X
AXIS_RY = 4   # Right stick Y
AXIS_LT = 2   # Left trigger (L2)
AXIS_RT = 5   # Right trigger (R2)

# Speed parameters (aligned with kinco_canopen_control.py)
SPEED_STEP  = 0.05        # Bước tăng/giảm tốc max (m/s)
SPEED_MIN   = 0.05
SPEED_MAX   = 2.0         # Tối đa 2.0 m/s
SPEED_INIT  = 0.50        # Tốc độ ban đầu 0.50 m/s
MAX_ANGULAR = 2.0         # Vận tốc góc tối đa (rad/s)

# Smooth ramp rates
ACCEL_RATE       = 0.5    # Gia tốc tuyến tính (m/s²)
DECEL_RATE       = 8.0    # Giảm tốc tuyến tính mềm hơn (m/s²)
ANGULAR_ACCEL    = 2.0    # Gia tốc góc (rad/s²)
ANGULAR_DECEL    = 6.0    # Giảm tốc góc MỀM → caster tự chỉnh trong lúc giảm (rad/s²)

SEND_HZ = 20  # Tần số gửi lệnh (Hz)

# Deadzone cho analog stick
DEADZONE = 0.15


def ramp_towards(current, target, accel_rate, decel_rate, dt):
    """
    Thay đổi giá trị current tiến dần về target với tốc độ tăng/giảm mượt mà.
    (Copied from kinco_canopen_control.py)
    """
    diff = target - current
    if abs(diff) < 0.001:
        return target

    # Chọn rate: giảm tốc nhanh hơn khi thả nút (target=0)
    if abs(target) < 0.001:
        rate = decel_rate
    else:
        rate = accel_rate

    step = rate * dt
    if abs(diff) <= step:
        return target
    elif diff > 0:
        return current + step
    else:
        return current - step


class FlydigiControllerNode(Node):
    def __init__(self):
        super().__init__('flydigi_controller_node')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.speed_pub_ = self.create_publisher(Float64, '/speed_setting', 10)

        # ROS parameter: chọn joystick theo index (-1 = tự động chọn cái đầu)
        self.declare_parameter('joystick_index', -1)
        joy_idx = self.get_parameter('joystick_index').value

        # Liệt kê tất cả joystick đang kết nối
        n_joy = pygame.joystick.get_count()
        self.get_logger().info(f"Tìm thấy {n_joy} tay cầm:")
        for i in range(n_joy):
            j = pygame.joystick.Joystick(i)
            j.init()
            self.get_logger().info(
                f"  [{i}] {j.get_name()} | Axes: {j.get_numaxes()}, Buttons: {j.get_numbuttons()}")

        # Chọn joystick
        if joy_idx < 0:
            # Tự động tìm tay cầm Flydigi
            joy_idx = self._find_flydigi(n_joy)
            if joy_idx < 0:
                self.get_logger().warn("Không tìm thấy tay cầm Flydigi! Dùng tay cầm đầu tiên.")
                joy_idx = 0

        if joy_idx >= n_joy:
            self.get_logger().error(f"Joystick index {joy_idx} không tồn tại! Chỉ có {n_joy} tay cầm.")
            raise RuntimeError(f"Joystick {joy_idx} not found")

        self.joystick = pygame.joystick.Joystick(joy_idx)
        self.joystick.init()
        joy_name = self.joystick.get_name()
        self.get_logger().info(f"▶ Đang dùng tay cầm [{joy_idx}]: {joy_name}")
        self.get_logger().info(f"  Số trục: {self.joystick.get_numaxes()}, Số nút: {self.joystick.get_numbuttons()}")

        # Detect axis mapping dựa trên số trục
        self._detect_axis_mapping()

        # Speed state
        self.max_linear = SPEED_INIT
        self._publish_speed()  # Publish tốc độ ban đầu
        self.rb_was_pressed = False
        self.lb_was_pressed = False

        # Smooth ramp: current velocities (change gradually)
        self.current_v = 0.0      # m/s
        self.current_omega = 0.0  # rad/s

        # Timer at SEND_HZ
        self.dt = 1.0 / SEND_HZ
        self.timer = self.create_timer(self.dt, self.timer_callback)

        self.get_logger().info(f"=== Điều khiển robot 2 bánh Kinco (Flydigi) ===")
        self.get_logger().info(f"Y: Tiến | A: Lùi | X: Quay trái | B: Quay phải")
        self.get_logger().info(f"Left Stick: Tiến/Lùi | Right Stick: Quay")
        self.get_logger().info(f"RB(R1): Tăng tốc | LB(L1): Giảm tốc | START: Thoát")
        self.get_logger().info(f"Tốc độ ban đầu: {self.max_linear:.2f} m/s")
        self.get_logger().info(f"Ramp: accel={ACCEL_RATE} m/s², decel={DECEL_RATE} m/s²")

    def _find_flydigi(self, n_joy):
        """Tự động tìm tay cầm Flydigi trong danh sách joystick."""
        flydigi_keywords = ['flydigi', 'vader', 'apex', 'direwolf', 'wee']
        for i in range(n_joy):
            j = pygame.joystick.Joystick(i)
            j.init()
            name_lower = j.get_name().lower()
            for keyword in flydigi_keywords:
                if keyword in name_lower:
                    self.get_logger().info(f"Tìm thấy tay cầm Flydigi tại index {i}: {j.get_name()}")
                    return i
        return -1

    def _detect_axis_mapping(self):
        """
        Detect axis mapping dựa trên số trục của tay cầm.
        Flydigi có thể report 6 hoặc 8 trục tùy model/firmware.
        """
        num_axes = self.joystick.get_numaxes()
        self.get_logger().info(f"Detecting axis mapping cho {num_axes} trục...")

        if num_axes >= 6:
            # Xinput standard: LX=0, LY=1, LT=2, RX=3, RY=4, RT=5
            self.axis_lx = 0
            self.axis_ly = 1
            self.axis_rx = 3
            self.axis_ry = 4
            self.get_logger().info("  Mapping: Xinput standard (6+ axes)")
        elif num_axes >= 4:
            # Compact mode: LX=0, LY=1, RX=2, RY=3
            self.axis_lx = 0
            self.axis_ly = 1
            self.axis_rx = 2
            self.axis_ry = 3
            self.get_logger().info("  Mapping: Compact (4 axes)")
        else:
            # Chỉ có 2 trục (left stick only)
            self.axis_lx = 0
            self.axis_ly = 1
            self.axis_rx = 0  # dùng chung left stick cho rotation
            self.axis_ry = -1
            self.get_logger().warn("  Chỉ có 2 trục! Dùng left stick cho cả di chuyển và quay.")

    def _publish_speed(self):
        """Publish tốc độ max hiện tại lên /speed_setting."""
        msg = Float64()
        msg.data = float(self.max_linear)
        self.speed_pub_.publish(msg)

    def timer_callback(self):
        pygame.event.pump()

        # --- Speed adjustment: RB = tăng tốc, LB = giảm tốc ---
        rb_pressed = self.joystick.get_button(BTN_RB)
        lb_pressed = self.joystick.get_button(BTN_LB)

        if rb_pressed and not self.rb_was_pressed:
            self.max_linear = min(self.max_linear + SPEED_STEP, SPEED_MAX)
            self.get_logger().info(f"Tăng tốc max: {self.max_linear:.2f} m/s")
            self._publish_speed()
        if lb_pressed and not self.lb_was_pressed:
            self.max_linear = max(self.max_linear - SPEED_STEP, SPEED_MIN)
            self.get_logger().info(f"Giảm tốc max: {self.max_linear:.2f} m/s")
            self._publish_speed()

        self.rb_was_pressed = rb_pressed
        self.lb_was_pressed = lb_pressed

        # --- Determine target velocities from buttons ---
        target_v = 0.0
        target_omega = 0.0

        # Button control (D-pad style)
        if self.joystick.get_button(BTN_Y):      # Y = tiến
            target_v = self.max_linear
        elif self.joystick.get_button(BTN_A):     # A = lùi
            target_v = -self.max_linear

        # Angular scales with max_linear (matching kinco_canopen_control.py)
        max_angular = MAX_ANGULAR * (self.max_linear / SPEED_INIT)
        max_angular = min(max_angular, MAX_ANGULAR * 4.0)  # safety cap

        if self.joystick.get_button(BTN_X):       # X = quay trái
            target_omega = max_angular
        elif self.joystick.get_button(BTN_B):     # B = quay phải
            target_omega = -max_angular

        # Analog stick control (only if buttons aren't driving)
        num_axes = self.joystick.get_numaxes()

        if abs(target_v) < 0.001:
            if num_axes > self.axis_ly:
                ax_ly = self.joystick.get_axis(self.axis_ly)
                if abs(ax_ly) > DEADZONE:
                    target_v = -ax_ly * self.max_linear

        if abs(target_omega) < 0.001:
            if num_axes > self.axis_rx:
                ax_rx = self.joystick.get_axis(self.axis_rx)
                if abs(ax_rx) > DEADZONE:
                    target_omega = -ax_rx * max_angular
            # Fallback: dùng left stick X nếu không có right stick riêng
            elif self.axis_rx == self.axis_lx and num_axes > self.axis_lx:
                ax_lx = self.joystick.get_axis(self.axis_lx)
                if abs(ax_lx) > DEADZONE:
                    target_omega = -ax_lx * max_angular

        # --- Smooth ramp towards target ---
        self.current_v = ramp_towards(
            self.current_v, target_v, ACCEL_RATE, DECEL_RATE, self.dt)
        self.current_omega = ramp_towards(
            self.current_omega, target_omega, ANGULAR_ACCEL, ANGULAR_DECEL, self.dt)

        # Publish Twist - Differential drive: only linear.x and angular.z
        msg = Twist()
        msg.linear.x = float(round(self.current_v, 4))
        msg.linear.y = 0.0  # Differential drive không thể đi ngang
        msg.angular.z = float(round(self.current_omega, 4))
        self.publisher_.publish(msg)

        # Exit on START button (smooth deceleration first)
        if self.joystick.get_numbuttons() > BTN_START and self.joystick.get_button(BTN_START):
            self.get_logger().info("Nhấn START - giảm tốc và thoát...")
            self._smooth_stop()
            stop_msg = Twist()
            self.publisher_.publish(stop_msg)
            raise SystemExit()

    def _smooth_stop(self):
        """Giảm tốc mượt mà về 0 trước khi thoát (matching kinco_canopen_control.py)."""
        for _ in range(int(SEND_HZ * 2)):  # tối đa 2 giây
            self.current_v = ramp_towards(
                self.current_v, 0, ACCEL_RATE, DECEL_RATE, self.dt)
            self.current_omega = ramp_towards(
                self.current_omega, 0, ANGULAR_ACCEL, ANGULAR_DECEL, self.dt)

            msg = Twist()
            msg.linear.x = float(round(self.current_v, 4))
            msg.angular.z = float(round(self.current_omega, 4))
            self.publisher_.publish(msg)

            if abs(self.current_v) < 0.001 and abs(self.current_omega) < 0.001:
                break
            import time
            time.sleep(self.dt)


def main(args=None):
    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print("Không tìm thấy tay cầm! Cắm USB receiver và thử lại.")
        print("Tip: Thử chạy 'jstest /dev/input/js0' để kiểm tra tay cầm.")
        pygame.quit()
        return

    rclpy.init(args=args)
    node = FlydigiControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Ctrl+C - giảm tốc...")
        try:
            node._smooth_stop()
            stop_msg = Twist()
            node.publisher_.publish(stop_msg)
        except Exception:
            pass
    except (SystemExit, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
        pygame.quit()


if __name__ == '__main__':
    main()
