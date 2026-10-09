#!/usr/bin/env python3
"""
Puente seguro entre control_bebop y ros2_bebop_driver (namespace /bebop).

Entradas (stack interno):
    /bridge/cmd_vel_raw  (Twist)   comandos normalizados [-1, 1]
    /bridge/takeoff, /bridge/land, /bridge/emergency, /bridge/flattrim (Empty)
Salidas (driver):
    /bebop/cmd_vel, /bebop/takeoff, /bebop/land, /bebop/reset, /bebop/flattrim
Odometría:
    /bebop/odom → /bebop1/pose (Pose, marco de despegue) y /odom_local (Odometry)
"""
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty

from .frames import TakeoffFrame, yaw_from_quaternion, quaternion_from_yaw
from .safety import GeofenceParams, apply_geofence, clamp


class RealDroneBridge(Node):
    def __init__(self):
        super().__init__('real_drone_bridge')

        # ── Parámetros ────────────────────────────────────────
        self.declare_parameter('max_z', 2.0)
        self.declare_parameter('arena_x_max', 2.0)
        self.declare_parameter('geofence_enabled', True)
        # Calibración: soporta tanto nombres antiguos (scale_x) como nuevos (odom_scale_x)
        self.declare_parameter('scale_x', 1.0)
        self.declare_parameter('scale_y', 1.0)
        self.declare_parameter('odom_scale_x', 1.0)
        self.declare_parameter('odom_scale_y', 1.0)
        self.declare_parameter('odom_bias_vx', 0.0)
        self.declare_parameter('odom_bias_vy', 0.0)
        self.declare_parameter('trim_x', 0.0)
        self.declare_parameter('trim_y', 0.0)
        self.declare_parameter('trim_pitch', 0.0)
        self.declare_parameter('trim_roll', 0.0)
        # Saturación de comandos enviados al driver
        self.declare_parameter('max_cmd_xy', 0.4)
        self.declare_parameter('max_cmd_z', 0.6)
        self.declare_parameter('max_cmd_yaw', 0.6)
        self.declare_parameter('cmd_timeout', 0.3)

        gp = self.get_parameter
        sx = gp('odom_scale_x').value if gp('odom_scale_x').value != 1.0 else gp('scale_x').value
        sy = gp('odom_scale_y').value if gp('odom_scale_y').value != 1.0 else gp('scale_y').value
        self.trim_pitch = gp('trim_pitch').value if gp('trim_pitch').value != 0.0 else gp('trim_x').value
        self.trim_roll = gp('trim_roll').value if gp('trim_roll').value != 0.0 else gp('trim_y').value

        self.geofence_params = GeofenceParams(
            enabled=gp('geofence_enabled').value,
            half_size=gp('arena_x_max').value,
            max_altitude=gp('max_z').value,
        )
        self.takeoff_frame = TakeoffFrame(
            scale=(sx, sy, 1.0),
            vel_bias=(gp('odom_bias_vx').value, gp('odom_bias_vy').value),
        )
        self.max_xy = gp('max_cmd_xy').value
        self.max_z = gp('max_cmd_z').value
        self.max_yaw = gp('max_cmd_yaw').value
        self.cmd_timeout = gp('cmd_timeout').value

        # ── Publishers al driver ──────────────────────────────
        self.pub_cmd_vel = self.create_publisher(Twist, '/bebop/cmd_vel', 10)
        self.pub_takeoff = self.create_publisher(Empty, '/bebop/takeoff', 10)
        self.pub_land = self.create_publisher(Empty, '/bebop/land', 10)
        self.pub_reset = self.create_publisher(Empty, '/bebop/reset', 10)
        self.pub_flattrim = self.create_publisher(Empty, '/bebop/flattrim', 10)

        # ── Publishers al stack interno ───────────────────────
        self.pub_odom_local = self.create_publisher(Odometry, '/odom_local', 10)
        self.pub_pose = self.create_publisher(Pose, '/bebop1/pose', 10)

        # ── Subscribers del stack interno ─────────────────────
        self.create_subscription(Twist, '/bridge/cmd_vel_raw', self.cmd_vel_cb, 10)
        self.create_subscription(Empty, '/bridge/takeoff', self.takeoff_cb, 10)
        self.create_subscription(Empty, '/bridge/land', self.land_cb, 10)
        self.create_subscription(Empty, '/bridge/emergency', self.emergency_cb, 10)
        self.create_subscription(Empty, '/bridge/flattrim', self.flattrim_cb, 10)

        # ── Subscriber del driver ─────────────────────────────
        self.create_subscription(Odometry, '/bebop/odom', self.odom_cb, 10)

        # ── Estado ────────────────────────────────────────────
        self.current_local_pose = None
        self.last_cmd_vel = Twist()
        self.last_cmd_time = 0.0
        self.last_odom_time = None
        self._last_geofence_log = 0.0

        self.create_timer(0.05, self.watchdog_loop)   # 20 Hz
        self.create_timer(5.0, self.status_loop)
        self.get_logger().info(
            'Real Drone Bridge listo. /bridge/* → /bebop/*  |  watchdog 20 Hz')

    # ── Comandos discretos ────────────────────────────────────
    def takeoff_cb(self, _):
        # Re-anclar el marco de despegue al punto actual
        self.takeoff_frame.origin = None
        self.pub_takeoff.publish(Empty())
        self.get_logger().info('🛫 TAKEOFF → /bebop/takeoff')

    def land_cb(self, _):
        self.pub_land.publish(Empty())
        self.get_logger().info('🛬 LAND → /bebop/land')

    def emergency_cb(self, _):
        self.pub_reset.publish(Empty())
        self.get_logger().error('🚨 EMERGENCY (corte de motores) → /bebop/reset')

    def flattrim_cb(self, _):
        self.pub_flattrim.publish(Empty())
        self.get_logger().info('📐 FLATTRIM → /bebop/flattrim')

    def cmd_vel_cb(self, msg):
        self.last_cmd_vel = msg
        self.last_cmd_time = time.time()

    # ── Odometría ─────────────────────────────────────────────
    def odom_cb(self, msg):
        self.last_odom_time = time.time()
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        lx, ly, lz, lyaw = self.takeoff_frame.pose_to_local(p.x, p.y, p.z, yaw, t)
        lqx, lqy, lqz, lqw = quaternion_from_yaw(lyaw)

        pose = Pose()
        pose.position.x = lx
        pose.position.y = ly
        pose.position.z = lz
        pose.orientation.x = lqx
        pose.orientation.y = lqy
        pose.orientation.z = lqz
        pose.orientation.w = lqw
        self.pub_pose.publish(pose)

        local = Odometry()
        local.header = msg.header
        local.header.frame_id = 'takeoff'
        local.child_frame_id = 'base_link'
        local.pose.pose = pose
        local.twist = msg.twist
        self.pub_odom_local.publish(local)

        self.current_local_pose = (lx, ly, lz, lyaw)

    # ── Watchdog ──────────────────────────────────────────────
    def watchdog_loop(self):
        cmd = Twist()   # ceros = hover nativo del Bebop
        now = time.time()

        if (now - self.last_cmd_time) < self.cmd_timeout:
            src = self.last_cmd_vel
            pitch = clamp(src.linear.x + self.trim_pitch, -self.max_xy, self.max_xy)
            roll = clamp(src.linear.y + self.trim_roll, -self.max_xy, self.max_xy)
            gaz = clamp(src.linear.z, -self.max_z, self.max_z)
            yaw_rate = clamp(src.angular.z, -self.max_yaw, self.max_yaw)

            if self.current_local_pose is not None:
                (pitch, roll, gaz, yaw_rate), events = apply_geofence(
                    (pitch, roll, gaz, yaw_rate),
                    self.current_local_pose, self.geofence_params)
                if events and now - self._last_geofence_log > 1.0:
                    self._last_geofence_log = now
                    self.get_logger().warn(f'Geocerca: {events}')

            cmd.linear.x = float(pitch)
            cmd.linear.y = float(roll)
            cmd.linear.z = float(gaz)
            cmd.angular.z = float(yaw_rate)

        self.pub_cmd_vel.publish(cmd)

    def status_loop(self):
        if self.last_odom_time is None:
            self.get_logger().warn(
                'Sin /bebop/odom todavía — ¿está corriendo el driver y conectado al Wi-Fi del dron?')
        elif time.time() - self.last_odom_time > 2.0:
            self.get_logger().warn('/bebop/odom detenido (>2 s)')


def main(args=None):
    rclpy.init(args=args)
    node = RealDroneBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Último mensaje: ceros (hover)
        node.pub_cmd_vel.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
