#!/usr/bin/env python3
"""IDS830 Motor Test Script - Interactive control with continuous encoder feedback."""

import socket
import struct
import time
import subprocess
import sys
import os
import threading
import select


class IDS830CAN:
    """IDS830 driver control via SocketCAN."""

    FUNC_WRITE = 0x1A
    FUNC_READ = 0x2A
    FUNC_READ_RESP = 0x2B
    FUNC_AUTO_REPORT = 0x88

    def __init__(self):
        self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.sock.bind(('can0',))
        self.sock.settimeout(0.5)
        self.lock = threading.Lock()

    def send(self, can_id, data):
        frame = struct.pack('=IB3x8s', can_id, len(data), bytes(data))
        self.sock.send(frame)

    def recv(self, timeout=0.5):
        """Receive CAN frame, skip auto-report."""
        end = time.time() + timeout
        while time.time() < end:
            try:
                frame = self.sock.recv(16)
                can_id, dlc = struct.unpack('=IB3x', frame[:8])
                data = list(frame[8:8 + dlc])
                if len(data) >= 2 and data[1] != self.FUNC_AUTO_REPORT:
                    return data
            except socket.timeout:
                break
        return None

    def unlock_pc_mode(self, can_id):
        """Unlock PC mode by clearing bit 5 of register 0x36."""
        with self.lock:
            self.send(can_id, [0x00, self.FUNC_READ, 0x36, 0x00, 0x00, 0xFF, 0x00, 0x00])
            r = self.recv()
            if r and r[1] == self.FUNC_READ_RESP:
                val = ((r[3] << 8) | r[4]) & ~(1 << 5)  # Clear bit 5
                self.send(can_id, [0x00, self.FUNC_WRITE, 0x36,
                                   (val >> 8) & 0xFF, val & 0xFF, 0xFF, 0x00, 0x00])
                self.recv()
                return True
        return False

    def set_speed_mode(self, can_id):
        """Set Speed Mode - PC Digital Input (0xC4)."""
        with self.lock:
            self.send(can_id, [0x00, self.FUNC_WRITE, 0x02, 0x00, 0xC4, 0xFF, 0x00, 0x00])
            return self.recv()

    def enable(self, can_id):
        """Enable motor."""
        with self.lock:
            self.send(can_id, [0x00, self.FUNC_WRITE, 0x00, 0x00, 0x01, 0xFF, 0x00, 0x00])
            return self.recv()

    def disable(self, can_id):
        """Disable motor."""
        with self.lock:
            self.send(can_id, [0x00, self.FUNC_WRITE, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x00])
            return self.recv()

    def set_speed(self, can_id, rpm):
        """Set speed in RPM. Negative = reverse."""
        speed_val = int((rpm / 3000.0) * 8192)
        if speed_val < 0:
            speed_val = (1 << 16) + speed_val
        speed_val &= 0xFFFF
        h = (speed_val >> 8) & 0xFF
        l = speed_val & 0xFF
        with self.lock:
            self.send(can_id, [0x00, self.FUNC_WRITE, 0x06, h, l, 0xFF, 0x00, 0x00])
            return self.recv()

    def read_encoder(self, can_id):
        """Read 32-bit encoder position."""
        with self.lock:
            self.send(can_id, [0x00, self.FUNC_READ, 0xE8, 0x00, 0x00, 0xE9, 0x00, 0x00])
            r = self.recv()
        if r and r[1] == self.FUNC_READ_RESP:
            pos = (r[3] << 24) | (r[4] << 16) | (r[6] << 8) | r[7]
            if pos & (1 << 31):
                pos -= 1 << 32
            return pos
        return None

    def read_speed(self, can_id):
        """Read speed feedback."""
        with self.lock:
            self.send(can_id, [0x00, self.FUNC_READ, 0xE4, 0x00, 0x00, 0xFF, 0x00, 0x00])
            r = self.recv()
        if r and r[1] == self.FUNC_READ_RESP:
            spd = (r[3] << 8) | r[4]
            if spd & (1 << 15):
                spd -= 1 << 16
            return (spd / 8192.0) * 3000
        return None

    def close(self):
        self.sock.close()


def setup_can():
    """Setup CAN interface."""
    os.system("sudo killall slcand 2>/dev/null")
    time.sleep(0.3)
    os.system("sudo slcand -o -s6 -t hw -S 921600 /dev/ttyACM0 can0 2>/dev/null")
    os.system("sudo ip link set up can0 2>/dev/null")
    time.sleep(0.3)


def encoder_thread(can, left_id, right_id, stop_event):
    """Background thread to continuously print encoder values."""
    while not stop_event.is_set():
        enc_l = can.read_encoder(left_id)
        enc_r = can.read_encoder(right_id)
        spd_l = can.read_speed(left_id)
        spd_r = can.read_speed(right_id)

        left_str = f"Enc={enc_l:>10d}  Spd={spd_l:>7.1f} RPM" if enc_l is not None and spd_l is not None else "---"
        right_str = f"Enc={enc_r:>10d}  Spd={spd_r:>7.1f} RPM" if enc_r is not None and spd_r is not None else "---"

        # Overwrite current line
        print(f"\r  TRÁI: {left_str}  |  PHẢI: {right_str}    ", end="", flush=True)
        stop_event.wait(0.2)  # Update 5 times/second


def main():
    LEFT_ID = 0x004
    RIGHT_ID = 0x002

    print("=" * 50)
    print("  IDS830 Motor Test")
    print("  Driver TRÁI: CAN ID 0x004")
    print("  Driver PHẢI: CAN ID 0x002")
    print("=" * 50)

    # Setup CAN
    print("\n[1/3] Setting up CAN interface...")
    setup_can()

    # Connect
    print("[2/3] Connecting...")
    can = IDS830CAN()

    # Unlock PC mode
    print("[3/3] Unlocking PC mode...")
    for cid, name in [(LEFT_ID, "TRÁI"), (RIGHT_ID, "PHẢI")]:
        ok = can.unlock_pc_mode(cid)
        can.set_speed_mode(cid)
        can.enable(cid)
        print(f"  Motor {name} (0x{cid:03X}): {'✅ OK' if ok else '❌ FAIL'}")

    # Start encoder thread
    stop_event = threading.Event()
    enc_thread = threading.Thread(target=encoder_thread, args=(can, LEFT_ID, RIGHT_ID, stop_event), daemon=True)
    enc_thread.start()

    print("\n" + "=" * 50)
    print("  ĐIỀU KHIỂN:")
    print("  w = Tiến   s = Lùi   x = Dừng")
    print("  [số] = Đặt tốc độ RPM (1-3000)")
    print("  q = Thoát")
    print("  Encoder hiển thị liên tục bên dưới")
    print("=" * 50)

    speed = 300  # Default RPM

    try:
        while True:
            print()  # New line before prompt (after encoder line)
            cmd = input(f"[Speed={speed} RPM] > ").strip().lower()

            if cmd == 'q':
                break
            elif cmd == 'w':
                can.set_speed(LEFT_ID, speed)
                can.set_speed(RIGHT_ID, -speed)   # Right motor mounted reversed
                print(f">>> TIẾN {speed} RPM")
            elif cmd == 's':
                can.set_speed(LEFT_ID, -speed)
                can.set_speed(RIGHT_ID, speed)    # Right motor mounted reversed
                print(f">>> LÙI {speed} RPM")
            elif cmd == 'x':
                can.set_speed(LEFT_ID, 0)
                can.set_speed(RIGHT_ID, 0)
                print(">>> DỪNG")
            elif cmd.isdigit() and 1 <= int(cmd) <= 3000:
                speed = int(cmd)
                print(f"  Tốc độ đặt: {speed} RPM")
            else:
                print("  Lệnh: w=tiến s=lùi x=dừng [1-3000]=RPM q=thoát")

    except KeyboardInterrupt:
        print("\n\nCtrl+C detected!")

    # Cleanup
    stop_event.set()
    enc_thread.join(timeout=1)
    print("\nDừng motor...")
    for cid in [LEFT_ID, RIGHT_ID]:
        can.set_speed(cid, 0)
        can.disable(cid)
    can.close()
    print("Done!")


if __name__ == "__main__":
    main()
