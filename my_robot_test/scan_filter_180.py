#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class LidarFront180(Node):
    def __init__(self):
        super().__init__('lidar_front_180')
        self.sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )
        self.pub = self.create_publisher(LaserScan, '/scan_180', 10)
        self.get_logger().info('Node /scan_180 starting')

    def scan_callback(self, msg: LaserScan):
        total_points = len(msg.ranges)
        if total_points < 180:
            self.get_logger().warn(' Few points available, try again :( )')
            return

        # Pegando os primeiros 90 pontos e os últimos 90 pontos
        front_ranges =  msg.ranges[-90:] + msg.ranges[:90] 
        front_intensities = msg.intensities[-90:] + msg.intensities[:90]

        # Criar nova mensagem
        filtered = LaserScan()
        filtered.header = msg.header
        filtered.ranges = front_ranges
        filtered.intensities = front_intensities
        filtered.range_min = msg.range_min
        filtered.range_max = msg.range_max
        filtered.scan_time = msg.scan_time
        filtered.time_increment = msg.time_increment
        filtered.angle_increment = msg.angle_increment
        filtered.angle_min = -math.pi / 2   # -90°
        filtered.angle_max = math.pi / 2    # +90°

        self.pub.publish(filtered)

def main(args=None):
    rclpy.init(args=args)
    node = LidarFront180()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Finishing node /scan_180...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
