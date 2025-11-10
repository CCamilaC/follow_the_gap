import rclpy
import numpy as np
from scipy.spatial.transform import Rotation as R
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PointStamped, TwistStamped
import math
from builtin_interfaces.msg import Time

# ============================================================
# =================== VARIÁVEIS GLOBAIS =====================
# ============================================================

global LA, wheel_base, points, car_yaw, car_global_axis

car_yaw = None
car_global_axis = None

# Lista inicial de pontos de destino (x, y)
points = [
    [1.52291, 0.029704]
]
#     [2.05358, -0.0236912],
wheel_base = 0.6
LA = 1.0  # Lookahead distance

# ============================================================
# =================== CALLBACKS =============================
# ============================================================

def clicked_point_callback(msg):
    """Adiciona pontos clicados no RViz à lista de trajetórias"""
    global points
    x = msg.point.x
    y = msg.point.y
    points.append([x, y])
    print(f"Ponto adicionado: ({x}, {y})")

def yaw_callback(odom):
    """Recebe a odometria do robô e atualiza posição e yaw"""
    global car_yaw, car_global_axis
    qx = odom.pose.pose.orientation.x
    qy = odom.pose.pose.orientation.y
    qz = odom.pose.pose.orientation.z
    qw = odom.pose.pose.orientation.w

    x = odom.pose.pose.position.x
    y = odom.pose.pose.position.y
    car_global_axis = [x, y]

    r = R.from_quat([qx, qy, qz, qw])
    _, _, car_yaw = r.as_euler('xyz')

# ============================================================
# =================== FUNÇÕES AUXILIARES =====================
# ============================================================

def choosing_point(points, car_pos, lookahead_distance=LA):
    """Escolhe o ponto da lista que está a distância de lookahead à frente do robô"""
    if not points or car_pos is None:
        return None
    for p in points:
        dist = math.sqrt((p[0] - car_pos[0])**2 + (p[1] - car_pos[1])**2)
        if dist >= lookahead_distance:
            return p
    return points[-1]

def distance(p1, p2):
    return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)

def order_points(points):
    """Ordena os pontos para seguir a trajetória de forma sequencial"""
    if car_global_axis is None:
        return points.copy()
    ordered = [car_global_axis]
    remaining = points.copy()
    while remaining:
        last = ordered[-1]
        nearest = min(remaining, key=lambda p: distance(p, last))
        ordered.append(nearest)
        remaining.remove(nearest)
    return ordered[1:]  # remove o ponto inicial duplicado

def point_global_to_local(point_global_axis, car_yaw, car_axis):
    """Transforma ponto global para coordenadas locais do robô"""
    dx = point_global_axis[0] - car_axis[0]
    dy = point_global_axis[1] - car_axis[1]

    point_local_x = dx * np.cos(car_yaw) + dy * np.sin(car_yaw)
    point_local_y = dy * np.cos(car_yaw) - dx * np.sin(car_yaw)
    return point_local_x, point_local_y

def calc_curv(point_local_y, lock_ahead=LA):
    """Calcula a curvatura para seguir o ponto"""
    curvature = (2 * point_local_y) / (lock_ahead**2)
    return curvature


def generate_command(curvature, linear_speed=0.2, node=None):
    angular_speed = curvature * linear_speed

    cmd_stamped = TwistStamped()
    cmd_stamped.header.stamp = node.get_clock().now().to_msg()
    cmd_stamped.header.frame_id = "base_link"
    cmd_stamped.twist.linear.x = float(linear_speed)
    cmd_stamped.twist.angular.z = float(angular_speed)
    return cmd_stamped


# ============================================================
# =================== MAIN ===================================
# ============================================================

def main(args=None):
    global car_yaw, car_global_axis

    rclpy.init(args=args)
    node = rclpy.create_node("pure_pursuit_node")

    # Subscrições
    node.create_subscription(Odometry, "/odom", yaw_callback, 10)
    node.create_subscription(PointStamped, "/clicked_point", clicked_point_callback, 10)

    # Publicador de velocidade
    vel_pub = node.create_publisher(TwistStamped, "/cmd_vel", 10)

    # Timer para envio de comandos
    def timer_callback():
        if car_yaw is None or car_global_axis is None:
            node.get_logger().warn("Waiting for odometry...")
            return

        arranged_points = order_points(points)
        selected_point = choosing_point(arranged_points, car_global_axis)
        if selected_point is None:
            return

        local_point_X, local_point_Y = point_global_to_local(selected_point, car_yaw, car_global_axis)
        curv = calc_curv(local_point_Y)
        cmd = generate_command(curvature=curv, linear_speed=0.2, node=node) 
        vel_pub.publish(cmd)

        node.get_logger().info(f"Vel: {cmd.twist.linear.x:.2f}, Angular: {cmd.twist.angular.z:.2f}") #change!


    timer = node.create_timer(0.1, timer_callback)
    rclpy.spin(node)

    # Encerramento
    node.destroy_timer(timer)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
