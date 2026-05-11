#!/usr/bin/env python3
"""
IDS830 Motor Test Script (Standalone - NO ROS2 needed)

Test CAN communication with IDS830 driver via SLCAN adapter.
This script lets you:
  1. Enable/Disable motor
  2. Set speed (RPM)
  3. Read encoder position
  4. Interactive control

Usage:
  python3 test_ids830.py                          # Default: /dev/ttyACM0, CAN ID 1
  python3 test_ids830.py --port /dev/ttyACM0 --id 1
  python3 test_ids830.py --port /dev/ttyACM0 --id 2   # Test right motor

Requirements:
  pip3 install pyserial
"""

import serial
import struct
import time
import argparse
import sys


class SLCAN:
    """SLCAN (Serial Line CAN) adapter interface."""

    def __init__(self, port, baudrate=921600, can_speed=6):
        """
        Args:
            port: Serial port (e.g., /dev/ttyACM0)
            baudrate: Serial baudrate to SLCAN adapter
            can_speed: CAN speed code (6=500kbps)
        """
        self.ser = serial.Serial(port, baudrate, timeout=0.05)
        time.sleep(0.1)

        # Close any existing session
        self._write("C\r")
        time.sleep(0.05)
        self.ser.reset_input_buffer()

        # Set CAN speed
        self._write(f"S{can_speed}\r")
        time.sleep(0.05)

        # Open CAN channel
        self._write("O\r")
        time.sleep(0.05)

        print(f"[SLCAN] Opened {port} at {baudrate} baud, CAN speed S{can_speed}")

    def _write(self, s):
        self.ser.write(s.encode())

    def send(self, can_id, data):
        """Send a CAN standard frame.

        Args:
            can_id: 11-bit CAN arbitration ID
            data: list of bytes (max 8)
        """
        frame = f"t{can_id:03X}{len(data)}"
        for b in data:
            frame += f"{b:02X}"
        frame += "\r"
        self._write(frame)

    def recv(self, timeout=0.1):
        """Receive a CAN frame.

        Returns:
            (can_id, data) tuple, or None if timeout
        """
        deadline = time.time() + timeout
        line = ""
        while time.time() < deadline:
            c = self.ser.read(1)
            if not c:
                continue
            c = c.decode(errors='ignore')
            if c in ('\r', '\n'):
                if line and line[0] == 't' and len(line) >= 5:
                    can_id = int(line[1:4], 16)
                    dlc = int(line[4])
                    data = []
                    for i in range(dlc):
                        data.append(int(line[5 + i*2:7 + i*2], 16))
                    return (can_id, data)
                line = ""
            else:
                line += c
        return None

    def close(self):
        self._write("C\r")
        time.sleep(0.05)
        self.ser.close()
        print("[SLCAN] Closed")


class IDS830:
    """IDS830 Low-Voltage DC Servo driver."""

    # Protocol constants
    GROUP = 0x00
    FUNC_WRITE = 0x1A
    FUNC_READ = 0x2A
    FUNC_READ_RESP = 0x2B
    REG_ENABLE = 0x00
    REG_SPEED = 0x06
    REG_ENCODER_HIGH = 0xE8
    REG_ENCODER_LOW = 0xE9
    REG_EMPTY = 0xFF

    def __init__(self, slcan, can_id):
        self.slcan = slcan
        self.can_id = can_id
        print(f"[IDS830] Motor CAN ID = 0x{can_id:02X}")

    def enable(self):
        """Enable motor."""
        data = [self.GROUP, self.FUNC_WRITE, self.REG_ENABLE,
                0x00, 0x01,  # Enable = 0x0001
                self.REG_EMPTY, 0x00, 0x00]
        self.slcan.send(self.can_id, data)
        print("[IDS830] Motor ENABLED")

    def disable(self):
        """Disable motor."""
        data = [self.GROUP, self.FUNC_WRITE, self.REG_ENABLE,
                0x00, 0x00,  # Disable = 0x0000
                self.REG_EMPTY, 0x00, 0x00]
        self.slcan.send(self.can_id, data)
        print("[IDS830] Motor DISABLED")

    def set_speed(self, rpm):
        """Set target speed in RPM.

        Formula: value = (RPM / 3000) * 8192
        Negative RPM = reverse direction.
        """
        # Convert RPM to IDS830 internal value
        speed_val = int((rpm / 3000.0) * 8192)

        # 16-bit two's complement
        if speed_val < 0:
            speed_val = (1 << 16) + speed_val
        speed_val &= 0xFFFF

        high = (speed_val >> 8) & 0xFF
        low = speed_val & 0xFF

        # Combined: Set speed (Reg 0x06) + Enable (Reg 0x00 = 0x0001)
        data = [self.GROUP, self.FUNC_WRITE, self.REG_SPEED,
                high, low,
                self.REG_ENABLE, 0x00, 0x01]
        self.slcan.send(self.can_id, data)

        actual_rpm = (speed_val if speed_val < 32768 else speed_val - 65536) * 3000 / 8192
        print(f"[IDS830] Speed set: {rpm} RPM (value=0x{speed_val:04X}, actual≈{actual_rpm:.1f} RPM)")

    def read_encoder(self):
        """Request and read encoder position.

        Returns:
            Encoder pulse count (signed 32-bit), or None if no response.
        """
        # Send read request
        data = [self.GROUP, self.FUNC_READ, self.REG_ENCODER_HIGH,
                0x00, 0x00,
                self.REG_ENCODER_LOW, 0x00, 0x00]
        self.slcan.send(self.can_id, data)

        # Wait for response
        resp = self.slcan.recv(timeout=0.05)
        if resp is None:
            return None

        resp_id, resp_data = resp
        if len(resp_data) < 8:
            return None

        # Verify read response
        if resp_data[1] != self.FUNC_READ_RESP:
            return None
        if resp_data[2] != self.REG_ENCODER_HIGH or resp_data[5] != self.REG_ENCODER_LOW:
            return None

        # Combine 4 bytes into signed 32-bit
        enc_high = (resp_data[3] << 8) | resp_data[4]
        enc_low = (resp_data[6] << 8) | resp_data[7]
        raw = (enc_high << 16) | enc_low

        # Signed 32-bit
        if raw & (1 << 31):
            raw -= 1 << 32

        return raw

    def stop(self):
        """Stop motor (set speed to 0, keep enabled)."""
        self.set_speed(0)
        print("[IDS830] Motor STOPPED")


def interactive_mode(motor):
    """Interactive control menu."""
    print("\n" + "="*50)
    print("IDS830 Interactive Motor Test")
    print("="*50)
    print("Commands:")
    print("  e        - Enable motor")
    print("  d        - Disable motor")
    print("  s <RPM>  - Set speed (e.g., 's 100', 's -200')")
    print("  r        - Read encoder once")
    print("  m        - Monitor encoder (continuous, Ctrl+C to stop)")
    print("  t        - Speed test (100 RPM for 2 seconds)")
    print("  0        - Stop motor (speed = 0)")
    print("  q        - Quit (stops motor first)")
    print("="*50)

    while True:
        try:
            cmd = input("\n> ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            cmd = "q"

        if not cmd:
            continue

        if cmd == 'e':
            motor.enable()

        elif cmd == 'd':
            motor.disable()

        elif cmd.startswith('s '):
            try:
                rpm = int(cmd.split()[1])
                if abs(rpm) > 3000:
                    print("WARNING: Max RPM is 3000!")
                    rpm = max(-3000, min(3000, rpm))
                motor.set_speed(rpm)
            except (ValueError, IndexError):
                print("Usage: s <RPM>  (e.g., 's 100')")

        elif cmd == 'r':
            enc = motor.read_encoder()
            if enc is not None:
                print(f"[Encoder] Position: {enc} pulses")
            else:
                print("[Encoder] No response! Check CAN connection.")

        elif cmd == 'm':
            print("Monitoring encoder... (Ctrl+C to stop)")
            prev = None
            try:
                while True:
                    enc = motor.read_encoder()
                    if enc is not None:
                        delta = enc - prev if prev is not None else 0
                        prev = enc
                        print(f"\r  Encoder: {enc:10d}  Delta: {delta:+6d}", end="", flush=True)
                    else:
                        print(f"\r  Encoder: NO RESPONSE          ", end="", flush=True)
                    time.sleep(0.05)
            except KeyboardInterrupt:
                print("\n  Stopped monitoring.")

        elif cmd == 't':
            print("Speed test: 100 RPM for 2 seconds...")
            motor.enable()
            time.sleep(0.1)

            # Read initial encoder
            enc_start = motor.read_encoder()
            print(f"  Encoder start: {enc_start}")

            # Run at 100 RPM
            motor.set_speed(100)
            time.sleep(2.0)
            motor.stop()

            # Read final encoder
            time.sleep(0.2)
            enc_end = motor.read_encoder()
            print(f"  Encoder end:   {enc_end}")

            if enc_start is not None and enc_end is not None:
                delta = enc_end - enc_start
                print(f"  Delta pulses:  {delta}")
                # At 100 RPM for 2 seconds = 200/60 = 3.33 revolutions
                # Expected pulses ≈ 3.33 * encoder_ppr
                print(f"  Expected ~3.33 revolutions")
                if delta != 0:
                    ppr_estimate = abs(delta) / 3.33
                    print(f"  Estimated encoder PPR ≈ {ppr_estimate:.0f}")

        elif cmd == '0':
            motor.stop()

        elif cmd == 'q':
            print("Stopping motor and quitting...")
            motor.stop()
            time.sleep(0.1)
            motor.disable()
            break

        else:
            print(f"Unknown command: '{cmd}'")


def main():
    parser = argparse.ArgumentParser(description='IDS830 Motor Test (Standalone)')
    parser.add_argument('--port', default='/dev/ttyACM0', help='SLCAN serial port')
    parser.add_argument('--baud', type=int, default=921600, help='Serial baudrate')
    parser.add_argument('--id', type=int, default=1, help='Motor CAN ID (1=left, 2=right)')
    parser.add_argument('--can-speed', type=int, default=6, help='CAN speed code (6=500kbps)')
    args = parser.parse_args()

    try:
        slcan = SLCAN(args.port, args.baud, args.can_speed)
    except Exception as e:
        print(f"ERROR: Cannot open {args.port}: {e}")
        print("Check:")
        print(f"  1. USB-CAN adapter plugged in?")
        print(f"  2. Correct port? (try: ls /dev/ttyACM* /dev/ttyUSB*)")
        print(f"  3. Permission? (try: sudo chmod 666 {args.port})")
        sys.exit(1)

    motor = IDS830(slcan, args.id)

    try:
        interactive_mode(motor)
    finally:
        slcan.close()


if __name__ == '__main__':
    main()
