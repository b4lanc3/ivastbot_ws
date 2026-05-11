#!/usr/bin/env python3
"""
Script test nhanh tay cầm Flydigi - không cần ROS2.
Hiển thị realtime trạng thái các nút và trục.
Chạy: python3 test_joystick.py
"""
import pygame
import time

pygame.init()
pygame.joystick.init()

n_joy = pygame.joystick.get_count()
print(f"\n=== TÌM THẤY {n_joy} TAY CẦM ===")

if n_joy == 0:
    print("Không tìm thấy tay cầm nào! Cắm USB và thử lại.")
    pygame.quit()
    exit(1)

# Init tất cả joystick để test
joysticks = []
for i in range(n_joy):
    j = pygame.joystick.Joystick(i)
    j.init()
    print(f"  [{i}] Tên: {j.get_name()}")
    print(f"       Axes: {j.get_numaxes()}, Buttons: {j.get_numbuttons()}, Hats: {j.get_numhats()}")
    joysticks.append(j)

print(f"\n=== NHẤN NÚT / GẠCHẨY TAY CẦM ĐỂ TEST (Ctrl+C để thoát) ===\n")

try:
    while True:
        pygame.event.pump()
        
        for idx, j in enumerate(joysticks):
            # Check buttons
            for b in range(j.get_numbuttons()):
                if j.get_button(b):
                    print(f"[Joy {idx}] BUTTON {b} PRESSED")
            
            # Check axes
            for a in range(j.get_numaxes()):
                val = j.get_axis(a)
                if abs(val) > 0.15:
                    print(f"[Joy {idx}] AXIS {a} = {val:.3f}")
            
            # Check hats (D-pad)
            for h in range(j.get_numhats()):
                hat = j.get_hat(h)
                if hat != (0, 0):
                    print(f"[Joy {idx}] HAT {h} = {hat}")
        
        time.sleep(0.1)

except KeyboardInterrupt:
    print("\nĐã thoát.")
    pygame.quit()
