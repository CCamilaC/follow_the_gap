#!/usr/bin/env python3
"""
ROS 2 NODE – FOLLOW-THE-GAP ALGORITHM USING REAL LIDAR DATA (/scan_180)
Versão com duas fases de movimento:
1) Rotaciona parado até o ângulo do gap detectado.
2) Move-se em linha reta até o centro do gap.
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Twist, TwistStamped


# ============================================================
# === ALGORITMO FOLLOW-THE-GAP ==============================
# ============================================================

def find_disparities(ranges, threshold):
    disparities = []
    for i in range(len(ranges) - 1):
        r1 = ranges[i] if not math.isinf(ranges[i]) else 100.0
        r2 = ranges[i + 1] if not math.isinf(ranges[i + 1]) else 100.0
        if abs(r2 - r1) > threshold:
            disparities.append(i)
    return disparities


def form_gaps_from_disparities(disparities, num_points):
    boundary_indices = [0] + [d + 1 for d in disparities] + [num_points]
    gaps = []
    for i in range(len(boundary_indices) - 1):
        start_idx = boundary_indices[i]
        end_idx = boundary_indices[i + 1] - 1
        gaps.append((start_idx, end_idx))
    return gaps


def compute_gap_width(ranges, start_idx, end_idx, angle_increment_deg):
    gap_points = ranges[start_idx:end_idx + 1]
    valid_points = [p for p in gap_points if not math.isinf(p) and p > 0.1]
    if not valid_points:
        return 0.0, 0.0, 0.0, 0.0

    min_dist = min(valid_points)
    avg_dist = sum(valid_points) / len(valid_points)
    num_points_in_gap = end_idx - start_idx + 1
    angular_width_deg = num_points_in_gap * angle_increment_deg
    angular_width_rad = math.radians(angular_width_deg)
    width = 2 * min_dist * math.tan(angular_width_rad / 2)
    return width, min_dist, angular_width_deg, avg_dist


def select_largest_gap(ranges, gaps, angle_increment_deg, rover_width):
    largest_width = 0.0
    selected_gap = None
    for (start_idx, end_idx) in gaps:
        width, min_dist, angular_width_deg, avg_dist = compute_gap_width(
            ranges, start_idx, end_idx, angle_increment_deg
        )
        traversable = width >= rover_width
        if traversable and width > largest_width:
            largest_width = width
            selected_gap = (start_idx, end_idx, width, min_dist, angular_width_deg, avg_dist)
    return selected_gap


def calculate_steering_angle(largest_gap, angle_min_deg, angle_increment_deg):
    if largest_gap is None:
        return None
    start_idx, end_idx = largest_gap[0], largest_gap[1]
    center_idx = (start_idx + end_idx) / 2.0
    return angle_min_deg + center_idx * angle_increment_deg


def follow_the_gap(lidar_ranges, angle_min_deg, angle_max_deg, rover_width=0.6, disparity_threshold=0.8):
    num_points = len(lidar_ranges)
    if num_points < 2:
        return {'status': 'STOP', 'steering_angle': None}
    angle_increment_deg = (angle_max_deg - angle_min_deg) / (num_points - 1)
    disparities = find_disparities(lidar_ranges, disparity_threshold)
    gaps = form_gaps_from_disparities(disparities, num_points)
    largest_gap = select_largest_gap(lidar_ranges, gaps, angle_increment_deg, rover_width)
    steering_angle = calculate_steering_angle(largest_gap, angle_min_deg, angle_increment_deg)

    if largest_gap:
        return {
            'status': 'SAFE',
            'steering_angle': steering_angle,
            'gap_width': largest_gap[2],
            'gap_distance': largest_gap[5],
            'gap_min_distance': largest_gap[3],
            'safety_margin': largest_gap[2] - rover_width,
        }
    else:
        return {'status': 'STOP', 'steering_angle': None}


# ============================================================
# === ROS 2 NODE =============================================
# ============================================================

class GapFollowerNode(Node):
    def __init__(self):
        super().__init__('gap_follower_node')

        # publishers / subscribers
        self.marker_pub = self.create_publisher(Marker, '/gap_marker', 10)
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan_180', self.scan_callback, 10)

        # parâmetros
        self.rover_width = 0.6
        self.disparity_threshold = 0.8
        self.linear_speed = 0.3  # m/s
        self.rotate_speed = 0.5  # rad/s
        self.marker_interval = 7.0
        self.last_marker_time = self.get_clock().now()

        # estado de movimento
        self.state = "IDLE"  # "ROTATING", "MOVING"
        self.moving = False
        self.move_end_time_sec = 0.0
        self.rotate_end_time_sec = 0.0
        self.rotate_direction = 1.0
        self.travel_time = 0.0
        self.current_steering_angle_deg = 0.0

        # cooldown
        self.last_move_time_sec = 0.0
        self.move_cooldown = 0.5

        # timer de movimento (10 Hz)
        self.motion_timer = self.create_timer(0.1, self.motion_timer_callback)

        self.get_logger().info('GapFollowerNode iniciado – ouvindo /scan_180')

    # --------------------------------------------------------
    def scan_callback(self, msg: LaserScan):
        angle_min_deg = math.degrees(msg.angle_min)
        angle_max_deg = math.degrees(msg.angle_max)
        ranges = list(msg.ranges)
        result = follow_the_gap(
            lidar_ranges=ranges,
            angle_min_deg=angle_min_deg,
            angle_max_deg=angle_max_deg,
            rover_width=self.rover_width,
            disparity_threshold=self.disparity_threshold
        )
        now_sec = self._now_seconds()

        # já em movimento?
        if self.moving or self.state != "IDLE":
            return

        if now_sec - self.last_move_time_sec < self.move_cooldown:
            return

        if result['status'] == 'SAFE':
            steering_angle = result['steering_angle']
            gap_distance = result['gap_distance']

            if gap_distance is None or gap_distance <= 0.0:
                self.get_logger().warn("Gap detectado, mas distância inválida. Ignorando.")
                return

            # tempo de translação
            travel_time = gap_distance / self.linear_speed
            travel_time = min(travel_time, 10.0)
            self.travel_time = travel_time

            # tempo de rotação
            target_angle_rad = math.radians(steering_angle)
            rotate_time = abs(target_angle_rad) / self.rotate_speed
            rotate_time = min(rotate_time, 3.0)
            self.rotate_direction = 1.0 if target_angle_rad > 0 else -1.0

            self.rotate_end_time_sec = now_sec + rotate_time
            self.current_steering_angle_deg = steering_angle
            self.state = "ROTATING"
            self.moving = True
            self.last_move_time_sec = now_sec

            self.get_logger().info(
                f"[SAFE] Ângulo: {steering_angle:.1f}°, "
                f"Largura: {result['gap_width']:.2f} m, "
                f"Dist. média: {gap_distance:.2f} m, "
                f"Margem: {result['safety_margin']:.2f} m"
            )
            self.get_logger().info(f"🔄 Rotacionando {steering_angle:.1f}° por {rotate_time:.2f}s")
            self.publish_marker_throttled(steering_angle, gap_distance)

        else:
            self.get_logger().warn("[STOP] Nenhum gap atravessável detectado!")
            self.stop_robot()

    # --------------------------------------------------------
    def motion_timer_callback(self):
        now_sec = self._now_seconds()

        if self.state == "ROTATING":
            if now_sec < self.rotate_end_time_sec:
                twist = TwistStamped()
                twist.header.stamp = self.get_clock().now().to_msg()
                twist.header.frame_id = 'base_link'
                twist.twist.angular.z = self.rotate_direction * self.rotate_speed
                self.cmd_pub.publish(twist)
            else:
                self.state = "MOVING"
                self.move_end_time_sec = now_sec + self.travel_time
                self.get_logger().info("✅ Rotação concluída – iniciando movimento linear.")

        elif self.state == "MOVING":
            if now_sec < self.move_end_time_sec:
                twist = TwistStamped()
                twist.header.stamp = self.get_clock().now().to_msg()
                twist.header.frame_id = 'base_link'
                twist.twist.linear.x = self.linear_speed
                self.cmd_pub.publish(twist)
            else:
                self.stop_robot()

        # estado IDLE -> nada

    # --------------------------------------------------------
    def stop_robot(self):
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = 'base_link'
        twist.twist.linear.x = 0.0
        twist.twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

        if self.moving:
            self.get_logger().info("🛑 Movimento concluído – robô parado.")

        self.state = "IDLE"
        self.moving = False
        self.last_move_time_sec = self._now_seconds()

    # --------------------------------------------------------
    def publish_marker_throttled(self, angle_deg, distance):
        now = self.get_clock().now()
        elapsed = (now - self.last_marker_time).nanoseconds * 1e-9
        if elapsed < self.marker_interval:
            return
        self.last_marker_time = now
        self.publish_marker(angle_deg, distance)

    # --------------------------------------------------------
    def publish_marker(self, angle_deg, distance):
        marker = Marker()
        marker.header.frame_id = 'base_scan'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'gap_direction'
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.lifetime.sec = 10

        length = 2.0
        angle_rad = math.radians(angle_deg)
        start = Point(x=0.0, y=0.0, z=0.0)
        end = Point(
            x=length * math.cos(angle_rad),
            y=length * math.sin(angle_rad),
            z=0.0
        )
        marker.points = [start, end]
        marker.scale.x = 0.2
        marker.scale.y = 0.4
        marker.scale.z = 0.4
        marker.color.a = 1.0
        marker.color.g = 1.0
        self.marker_pub.publish(marker)
        self.get_logger().info(f"✅ Marker publicado em {marker.header.frame_id}, ângulo {angle_deg:.1f}°")

    # --------------------------------------------------------
    def _now_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9


# ============================================================
# === MAIN ===================================================
# ============================================================

def main(args=None):
    rclpy.init(args=args)
    node = GapFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt recebido — encerrando...")
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
