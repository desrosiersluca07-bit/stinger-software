'''
lidar obstacle detection
'''
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
from stinger_msgs.msg import Obstacle

class ObstacleDetection(Node):
    def __init__(self):
        super().__init__("Obstacle_Detection")

        # Parameters (change with: --ros-args -p name:=value)
        self.declare_parameter('scan_topic', '/stinger/laser/scan')
        # scan angle (rad) that points out the bow. 0.0 matches the URDF, try 3.14159 if the LiDAR is mounted backwards
        self.declare_parameter('forward_angle', 0.0)
        self.declare_parameter('half_angle', 0.45)  # half width of the cone checked (rad, about 25 deg)
        self.declare_parameter('max_distance', 0.8)  # only obstacles closer than this (m) count
        self.declare_parameter('min_points', 3)  # beams needed in the cone, ignores single noisy points

        self.forward_angle = self.get_parameter('forward_angle').get_parameter_value().double_value
        self.half_angle = self.get_parameter('half_angle').get_parameter_value().double_value
        self.max_distance = self.get_parameter('max_distance').get_parameter_value().double_value
        self.min_points = self.get_parameter('min_points').get_parameter_value().integer_value
        scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value

        self.scan_sub = self.create_subscription(LaserScan, scan_topic, self.scan_callback, 10)
        self.obstacle_pub = self.create_publisher(Obstacle, '/stinger/obstacle', 10)
        self.get_logger().info("Obstacle detection node initialized!")

    def find_obstacle(self, ranges, angle_min, angle_increment, range_min):
        """Return (detected, distance, angle) for the closest point in the cone ahead."""
        ranges = np.array(ranges, dtype=float)
        angles = angle_min + np.arange(len(ranges)) * angle_increment
        # angle of each beam relative to the bow, wrapped to [-pi, pi], positive = left
        rel = np.arctan2(np.sin(angles - self.forward_angle), np.cos(angles - self.forward_angle))

        valid = np.isfinite(ranges) & (ranges > range_min) & (ranges < self.max_distance) \
                & (np.abs(rel) < self.half_angle)

        if np.count_nonzero(valid) < self.min_points:
            return False, -1.0, 0.0

        closest = np.argmin(np.where(valid, ranges, np.inf))
        return True, float(ranges[closest]), float(rel[closest])

    def scan_callback(self, msg: LaserScan):
        detected, distance, angle = self.find_obstacle(
            msg.ranges, msg.angle_min, msg.angle_increment, msg.range_min)

        out = Obstacle()
        out.detected = detected
        out.distance = distance
        out.angle = angle
        self.obstacle_pub.publish(out)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()