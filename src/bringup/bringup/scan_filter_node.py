"""
Scan filter node: removes laser points that hit the robot body (self-scan).

The LiDAR is a 360° scanner mounted on the robot. When scanning backwards,
it sees parts of the robot body and reports them as obstacles.
This node filters out those self-scan points.

Robot size: 90cm (x) × 50cm (y) × 35cm (z)
Laser position relative to base_link: x=0.4, y=0, z=0.125

In laser_frame coordinates, the robot body occupies:
  x: [-0.85, 0.05]   (behind and slightly in front of laser)
  y: [-0.25, 0.25]    (left and right)

Any laser return falling inside this rectangle is the robot itself.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math


class ScanFilterNode(Node):
    def __init__(self):
        super().__init__('scan_filter_node')

        # Robot footprint parameters (meters)
        self.declare_parameter('robot_length', 0.9)        # 90cm in x
        self.declare_parameter('robot_width', 0.5)         # 50cm in y
        self.declare_parameter('laser_offset_x', 0.4)      # laser 40cm forward from center
        self.declare_parameter('laser_offset_y', 0.0)
        self.declare_parameter('footprint_margin', 0.05)   # 5cm extra margin
        self.declare_parameter('min_range', 0.10)          # absolute min range filter (10cm)

        robot_length = self.get_parameter('robot_length').value
        robot_width = self.get_parameter('robot_width').value
        laser_offset_x = self.get_parameter('laser_offset_x').value
        laser_offset_y = self.get_parameter('laser_offset_y').value
        margin = self.get_parameter('footprint_margin').value
        self.min_range = self.get_parameter('min_range').value

        # Robot footprint bounds in laser_frame coordinates
        # base_link center is at (-laser_offset_x, -laser_offset_y) in laser_frame
        # Robot extends ±length/2 in x and ±width/2 in y from base_link
        half_l = robot_length / 2.0 + margin
        half_w = robot_width / 2.0 + margin

        self.x_min = -half_l - laser_offset_x   # = -0.50 - 0.40 = -0.90
        self.x_max =  half_l - laser_offset_x   # = +0.50 - 0.40 = +0.10
        self.y_min = -half_w - laser_offset_y    # = -0.30
        self.y_max =  half_w - laser_offset_y    # = +0.30

        # Pre-compute the max self-scan range (diagonal distance to farthest footprint corner)
        corners = [
            (self.x_min, self.y_min),
            (self.x_min, self.y_max),
            (self.x_max, self.y_min),
            (self.x_max, self.y_max),
        ]
        self.max_self_range = max(math.sqrt(cx*cx + cy*cy) for cx, cy in corners)

        self.get_logger().info(
            f"Scan filter active — footprint in laser_frame: "
            f"x=[{self.x_min:.3f}, {self.x_max:.3f}], "
            f"y=[{self.y_min:.3f}, {self.y_max:.3f}], "
            f"max_self_range={self.max_self_range:.3f}m, "
            f"min_range={self.min_range:.3f}m"
        )

        self.sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.pub = self.create_publisher(
            LaserScan, '/scan_filtered', 10)

        self.get_logger().info("Scan filter node started")

    def scan_callback(self, msg: LaserScan):
        filtered = LaserScan()
        filtered.header = msg.header
        filtered.angle_min = msg.angle_min
        filtered.angle_max = msg.angle_max
        filtered.angle_increment = msg.angle_increment
        filtered.time_increment = msg.time_increment
        filtered.scan_time = msg.scan_time
        filtered.range_min = msg.range_min
        filtered.range_max = msg.range_max
        filtered.intensities = list(msg.intensities)

        filtered_ranges = list(msg.ranges)
        angle = msg.angle_min

        for i in range(len(filtered_ranges)):
            r = filtered_ranges[i]

            # Filter 1: absolute minimum range (too close = noise or self-scan)
            if r < self.min_range:
                filtered_ranges[i] = float('inf')
                angle += msg.angle_increment
                continue

            # Only check footprint for points that could be self-scan
            # (within max diagonal distance of robot footprint)
            if r <= self.max_self_range and msg.range_min <= r <= msg.range_max:
                x = r * math.cos(angle)
                y = r * math.sin(angle)

                # Filter 2: point inside robot footprint rectangle
                if (self.x_min <= x <= self.x_max and
                        self.y_min <= y <= self.y_max):
                    filtered_ranges[i] = float('inf')

            angle += msg.angle_increment

        filtered.ranges = filtered_ranges
        self.pub.publish(filtered)


def main(args=None):
    rclpy.init(args=args)
    node = ScanFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
