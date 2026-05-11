#!/usr/bin/env python3
"""
RPLidar A2M8 test script for ivastbot_ws.
Subscribes to /scan and prints live stats. Launches RViz2 automatically.

Usage:
    source /opt/ros/humble/setup.bash
    python3 test_lidar.py
"""

import math
import os
import subprocess
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

RVIZ_CONFIG = os.path.join(os.path.dirname(__file__), 'test_lidar.rviz')


class LidarTest(Node):
    def __init__(self):
        super().__init__('lidar_test')
        self._scan_count = 0
        self._last_stamp = None

        self.sub = self.create_subscription(
            LaserScan, '/scan', self._cb, 10)

        self.get_logger().info('Waiting for /scan ...')

    def _cb(self, msg: LaserScan):
        self._scan_count += 1

        ranges = [r for r in msg.ranges
                  if msg.range_min <= r <= msg.range_max and not math.isinf(r)]

        # Scan frequency from header timestamps
        freq_str = ''
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self._last_stamp is not None:
            dt = stamp - self._last_stamp
            if dt > 0:
                freq_str = f'  freq={1/dt:.1f} Hz'
        self._last_stamp = stamp

        total = len(msg.ranges)
        valid = len(ranges)

        if valid == 0:
            print(f'[#{self._scan_count:04d}] No valid points!{freq_str}')
            return

        min_r = min(ranges)
        max_r = max(ranges)
        avg_r = sum(ranges) / valid

        # Closest point direction
        min_idx = msg.ranges.index(min_r)
        min_angle = math.degrees(msg.angle_min + min_idx * msg.angle_increment)

        print(
            f'[#{self._scan_count:04d}] '
            f'valid={valid}/{total}  '
            f'min={min_r:.3f}m @ {min_angle:.1f}°  '
            f'max={max_r:.3f}m  '
            f'avg={avg_r:.3f}m'
            f'{freq_str}'
        )


def main():
    rclpy.init()
    node = LidarTest()

    rviz_cmd = ['rviz2', '-d', RVIZ_CONFIG] if os.path.exists(RVIZ_CONFIG) else ['rviz2']
    rviz = subprocess.Popen(rviz_cmd)
    node.get_logger().info(f'RViz2 started (pid {rviz.pid})')

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rviz.terminate()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
