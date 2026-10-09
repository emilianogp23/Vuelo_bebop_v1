import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
import math


def euler_from_quaternion(x, y, z, w):
    """Extrae roll, pitch, yaw de un cuaternión."""
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    t2 = max(-1.0, min(+1.0, +2.0 * (w * y - z * x)))
    pitch = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return roll, pitch, yaw


class OdomViewer(Node):
    """Convierte Pose → Odometry y loguea estado del dron.

    Suscribe a /bebop1/pose y:
    1. Publica nav_msgs/Odometry en /bebop1/odom (para rqt_plot)
    2. Loguea posición, orientación y velocidad estimada
    """

    def __init__(self):
        super().__init__('odom_viewer')

        self.declare_parameter('log_rate', 2.0)
        self.log_rate = self.get_parameter('log_rate').value

        # Subscriber
        self.sub = self.create_subscription(
            Pose, '/bebop1/pose', self.pose_callback, 10
        )

        # Publisher Odometry
        self.pub_odom = self.create_publisher(Odometry, '/bebop1/odom', 10)

        # Estado para cálculo de velocidad
        self.prev_x = 0.0
        self.prev_y = 0.0
        self.prev_z = 0.0
        self.prev_time = None
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.current_pose = None

        # Timer para logging
        self.log_timer = self.create_timer(1.0 / self.log_rate, self.log_state)

        self.get_logger().info('Visor de odometría iniciado')
        self.get_logger().info('Para graficar posición:')
        self.get_logger().info(
            '  rqt_plot /bebop1/odom/pose/pose/position/x'
            ' /bebop1/odom/pose/pose/position/y'
            ' /bebop1/odom/pose/pose/position/z'
        )
        self.get_logger().info('Para graficar velocidad:')
        self.get_logger().info(
            '  rqt_plot /bebop1/odom/twist/twist/linear/x'
            ' /bebop1/odom/twist/twist/linear/y'
            ' /bebop1/odom/twist/twist/linear/z'
        )

    def pose_callback(self, msg):
        """Recibe pose y calcula velocidades por diferencias finitas."""
        self.current_pose = msg
        now = self.get_clock().now()

        # Calcular velocidades por diferencias finitas
        if self.prev_time is not None:
            dt = (now - self.prev_time).nanoseconds / 1e9
            if dt > 1e-6:
                self.vx = (msg.position.x - self.prev_x) / dt
                self.vy = (msg.position.y - self.prev_y) / dt
                self.vz = (msg.position.z - self.prev_z) / dt

        self.prev_x = msg.position.x
        self.prev_y = msg.position.y
        self.prev_z = msg.position.z
        self.prev_time = now

        # Publicar como Odometry
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'world'
        odom.child_frame_id = 'bebop1/body'
        odom.pose.pose = msg
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.linear.z = self.vz
        self.pub_odom.publish(odom)

    def log_state(self):
        """Loguea posición, yaw y velocidad periódicamente."""
        if self.current_pose is None:
            self.get_logger().warn('Esperando datos de pose...')
            return

        p = self.current_pose
        q = p.orientation
        _, _, yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)

        self.get_logger().info(
            f'POS: x={p.position.x:+7.3f}  y={p.position.y:+7.3f}  '
            f'z={p.position.z:+7.3f}  |  '
            f'YAW: {math.degrees(yaw):+6.1f}°  |  '
            f'VEL: vx={self.vx:+5.2f}  vy={self.vy:+5.2f}  vz={self.vz:+5.2f} m/s'
        )


def main(args=None):
    rclpy.init(args=args)
    node = OdomViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

