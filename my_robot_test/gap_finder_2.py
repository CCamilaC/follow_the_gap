#!/usr/bin/env python3
"""
ROS 2 NODE – FOLLOW-THE-GAP ALGORITHM USING REAL LIDAR DATA (/scan_180)
Author: você :)
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

# ============================================================
# === ALGORTIMO FOLLOW-THE-GAP (igual ao seu código base) ====
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

def select_largest_gap(ranges, gaps, angle_increment_deg, rover_width, angle_min_deg=-90.0):
    """
    Select the best gap considering not only the width, but also the direction.
    Central gaps (close to 0°) have priority.
    """
    best_score = -1.0
    selected_gap = None

    for (start_idx, end_idx) in gaps:
        width, min_dist, angular_width_deg, avg_dist = compute_gap_width(
            ranges, start_idx, end_idx, angle_increment_deg
        )

        # Ignora gaps pequenos
        if width < rover_width:
            continue

        # Ângulo central do gap
        center_idx = (start_idx + end_idx) / 2
        center_angle = angle_min_deg + center_idx * angle_increment_deg

        # Peso angular: favorece gaps próximos ao centro (0°)
        angle_weight = math.cos(math.radians(center_angle))

        # Score final ponderado (0 se o gap estiver a 90°)
        score = width * max(angle_weight, 0.0)

        if score > best_score:
            best_score = score
            selected_gap = (start_idx, end_idx, width, min_dist, angular_width_deg, avg_dist)

    return selected_gap


def calculate_steering_angle(largest_gap, angle_min_deg, angle_increment_deg):
    if largest_gap is None:
        return None
    start_idx, end_idx = largest_gap[0], largest_gap[1]
    center_idx = (start_idx + end_idx) / 2
    return angle_min_deg + center_idx * angle_increment_deg

def follow_the_gap(lidar_ranges, angle_min_deg, angle_max_deg, rover_width=0.6, disparity_threshold=0.8):
    num_points = len(lidar_ranges)
    angle_increment_deg = (angle_max_deg - angle_min_deg) / (num_points - 1)
    disparities = find_disparities(lidar_ranges, disparity_threshold)
    gaps = form_gaps_from_disparities(disparities, num_points)
    largest_gap = select_largest_gap(lidar_ranges, gaps, angle_increment_deg, rover_width, angle_min_deg)
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
        self.marker_pub = self.create_publisher(Marker, '/gap_marker', 10)

        # Parâmetros
        self.rover_width = 0.6
        self.disparity_threshold = 0.8

        # Controle de tempo para publicar a seta a cada 7 segundos
        self.last_marker_time = self.get_clock().now()  # modificação 1
        self.marker_interval = 7.0  # modificação 2

        # Inscrição no tópico do LiDAR
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan_180',
            self.scan_callback,
            10
        )
        self.get_logger().info('GapFollowerNode iniciado – ouvindo /scan_180')

    # --------------------------------------------------------
    def scan_callback(self, msg: LaserScan):
        # Converte campos do LaserScan
        angle_min_deg = math.degrees(msg.angle_min)
        angle_max_deg = math.degrees(msg.angle_max)
        ranges = list(msg.ranges)

        # Executa o algoritmo
        result = follow_the_gap(
            lidar_ranges=ranges,
            angle_min_deg=angle_min_deg,
            angle_max_deg=angle_max_deg,
            rover_width=self.rover_width,
            disparity_threshold=self.disparity_threshold
        )

        # Impressão no terminal
        if result['status'] == 'SAFE':
            self.get_logger().info(
                f"[SAFE] Ângulo: {result['steering_angle']:.1f}°, "
                f"Largura: {result['gap_width']:.2f} m, "
                f"Distância média: {result['gap_distance']:.2f} m, "
                f"Margem: {result['safety_margin']:.2f} m"
            )
        else:
            self.get_logger().warn("[STOP] Nenhum gap atravessável detectado!")

        # Publica o marker a cada 7 segundos (modificação 3 e 4)
        if result['status'] == 'SAFE':
            self.publish_marker_throttled(result['steering_angle'], result['gap_distance'])

    # --------------------------------------------------------
    def publish_marker_throttled(self, angle_deg, distance):
        now = self.get_clock().now()
        elapsed = (now - self.last_marker_time).nanoseconds * 1e-9
        if elapsed < self.marker_interval:
            return  # ainda não passou 7 segundos
        self.last_marker_time = now
        self.publish_marker(angle_deg, distance)

    # --------------------------------------------------------
    def publish_marker(self, angle_deg, distance):
        marker = Marker()
        marker.header.frame_id = 'base_scan'  # use o mesmo que está no RViz!
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'gap_direction'
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        # Faz o marker durar 10 s (tempo suficiente pra enxergar)
        marker.lifetime.sec = 10  

        # Força o comprimento da seta pra facilitar ver (2 m)
        length = 2.0
        angle_rad = math.radians(angle_deg)

        start = Point()
        start.x = 0.0
        start.y = 0.0
        start.z = 0.0

        end = Point()
        end.x = length * math.cos(angle_rad)
        end.y = length * math.sin(angle_rad)
        end.z = 0.0

        marker.points = [start, end]

        # Tamanhos maiores pra ficar visível no RViz
        marker.scale.x = 0.2  # haste da seta
        marker.scale.y = 0.4  # largura da ponta
        marker.scale.z = 0.4  # altura da ponta

        # Cor verde bem visível
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        self.marker_pub.publish(marker)
        self.get_logger().info(f"✅ Marker publicado em {marker.header.frame_id}, ângulo {angle_deg:.1f}°")


# ============================================================
# === MAIN ===================================================
# ============================================================

def main(args=None):
    rclpy.init(args=args)
    node = GapFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
