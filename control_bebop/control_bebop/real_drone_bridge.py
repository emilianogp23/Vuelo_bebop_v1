#!/usr/bin/env python3
"""
Puente seguro entre control_bebop y ros2_bebop_driver (namespace /bebop).

Entradas (stack interno):
    /bridge/cmd_vel_raw  (Twist)   comandos normalizados [-1, 1]
Geocerca (CommandGate de safety.py): de /bridge/takeoff a /bridge/land o emergency,
    sobre todo comando y sobre los ceros del watchdog, con la pose corregida por
    visión si hay lock (geofence_use_vision) y la velocidad medida del Bebop.
    /bridge/takeoff, /bridge/land, /bridge/emergency, /bridge/flattrim (Empty)
Salidas (driver):
    /bebop/cmd_vel, /bebop/takeoff, /bebop/land, /bebop/reset, /bebop/flattrim
Odometría:
    /bebop/odom → /bebop1/pose (Pose, marco de despegue) y /odom_local (Odometry)
    /bebop/altitude (Float64, altura medida por el Bebop) → z de /bebop1/pose
Visión (use_vision, ver vision.py):
    /vision/onboard/pose (ArUco vistos por el Bebop) → corrige el offset de la odometría
    vision_shadow=false: /bebop1/pose en marco corral (corregida)
    vision_shadow=true:  /bebop1/pose igual que sin visión; la corregida va a /vision/fused_pose
    Con la visión al mando, la odometría sin corregir sale en /vision/odom_pose (para el bag).
    /vision/lock (Bool, 10 Hz): incertidumbre de la posición < vision_max_sigma
    /vision/reset (Empty: la pose saltó al inicializar con la primera medición)
"""
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Empty, Float64

from .frames import (TakeoffFrame, quaternion_from_yaw, select_altitude,
                     yaw_from_quaternion)
from .safety import CommandGate, GeofenceParams, clamp
from .vision import VisionFusion


def vision_update(fusion, msg, stats, z=None):
    """PoseWithCovarianceStamped de aruco_localizer → fusion; cuenta el resultado en stats.

    ``z``: altura medida del dron (None si no hay), para descartar PnP con z imposible.
    """
    p = msg.pose.pose
    meas = (p.position.x, p.position.y,
            yaw_from_quaternion(p.orientation.x, p.orientation.y,
                                p.orientation.z, p.orientation.w))
    cov = msg.pose.covariance
    res = fusion.vision(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, meas,
                        math.sqrt(max(cov[0], 1e-6)), math.sqrt(max(cov[35], 1e-6)),
                        None if z is None else p.position.z - z)
    stats[res] = stats.get(res, 0) + 1
    return res


def vision_status(fusion, stats, shadow):
    """Línea de log cada 5 s; vacía stats."""
    dx, dy, dyaw = fusion.filter.offset
    line = (f'Visión {"(sombra) " if shadow else ""}{dict(stats)} '
            f'offset ({dx:+.2f}, {dy:+.2f}) m {math.degrees(dyaw):+.1f}°, '
            f'σ {fusion.filter.sigma_xy:.2f} m {"LOCK" if fusion.locked else "sin lock"}')
    stats.clear()
    return line


def vision_fusion_from_params(node):
    """VisionFusion con los parámetros vision_* (también la usa sim_odometry)."""
    for name, default in (
            ('vision_q_xy', 0.0025),             # m²/s de deriva de la odometría
            ('vision_q_move', 0.0),              # m²/m: deriva por distancia recorrida
            ('vision_q_yaw', 1e-4),              # rad²/s
            ('vision_gate_sigma', 3.0),          # rechazo a más de N desviaciones
            ('vision_max_sigma', 0.20),          # m; arriba de esto no hay lock
            ('vision_init_sigma_xy', 0.05),      # m; colocación a mano en el origen
            ('vision_init_sigma_yaw_deg', 3.0),
            ('vision_offset_rate_xy', 0.10),     # m/s; suaviza las correcciones
            ('vision_offset_rate_yaw_deg', 5.0),
            ('vision_max_dz', 0.30)):            # m; z visual vs altura medida
        node.declare_parameter(name, default)
    gp = lambda n: node.get_parameter(n).value  # noqa: E731
    return VisionFusion(
        max_sigma=gp('vision_max_sigma'),
        max_dz=gp('vision_max_dz'),
        init_sigma_xy=gp('vision_init_sigma_xy'),
        init_sigma_yaw=math.radians(gp('vision_init_sigma_yaw_deg')),
        q_xy=gp('vision_q_xy'), q_yaw=gp('vision_q_yaw'), q_move=gp('vision_q_move'),
        gate_sigma=gp('vision_gate_sigma'),
        rate_xy=gp('vision_offset_rate_xy'),
        rate_yaw=math.radians(gp('vision_offset_rate_yaw_deg')))


def geofence_params_from_node(node):
    """GeofenceParams con los parámetros de real_drone.yaml (también la usa sim_odometry)."""
    for name, default in (
            ('geofence_enabled', True),
            ('arena_x_max', 2.0),
            ('max_z', 2.0),
            ('z_geofence_enabled', True),           # False: no actúa sobre gaz
            # Zona segura de despegue (círculo en el origen): fuera de ella no se
            # baja de min_z_outside_safe (robots en el piso). 0 = desactivado.
            ('safe_zone_radius', 0.5),
            ('min_z_outside_safe', 1.0),
            ('geofence_lookahead', 2.5),            # s; ver safety.GeofenceParams
            ('geofence_brake_gain', 2.0),
            ('geofence_push_gain', 0.5),
            ('geofence_max_push', 0.4),
            # Con visión en modo sombra, la geocerca usa la pose corregida si hay lock
            ('geofence_use_vision', True)):
        node.declare_parameter(name, default)
    gp = lambda n: node.get_parameter(n).value  # noqa: E731
    return GeofenceParams(
        enabled=gp('geofence_enabled'), half_size=gp('arena_x_max'),
        max_altitude=gp('max_z'), z_enabled=gp('z_geofence_enabled'),
        safe_zone_radius=gp('safe_zone_radius'),
        min_z_outside_safe=gp('min_z_outside_safe'),
        lookahead=gp('geofence_lookahead'), brake_gain=gp('geofence_brake_gain'),
        push_gain=gp('geofence_push_gain'), max_push=gp('geofence_max_push'))


class RealDroneBridge(Node):
    def __init__(self):
        super().__init__('real_drone_bridge')

        # ── Parámetros ────────────────────────────────────────
        geofence = geofence_params_from_node(self)
        # z de /bebop/altitude (sonar + barómetro) en vez de la vz integrada
        self.declare_parameter('use_altitude_topic', True)
        self.declare_parameter('altitude_timeout', 1.0)
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
        # Corrección visual
        self.declare_parameter('use_vision', False)
        self.declare_parameter('vision_shadow', True)

        gp = self.get_parameter
        sx = gp('odom_scale_x').value if gp('odom_scale_x').value != 1.0 else gp('scale_x').value
        sy = gp('odom_scale_y').value if gp('odom_scale_y').value != 1.0 else gp('scale_y').value
        self.trim_pitch = gp('trim_pitch').value if gp('trim_pitch').value != 0.0 else gp('trim_x').value
        self.trim_roll = gp('trim_roll').value if gp('trim_roll').value != 0.0 else gp('trim_y').value

        self.takeoff_frame = TakeoffFrame(
            scale=(sx, sy, 1.0),
            vel_bias=(gp('odom_bias_vx').value, gp('odom_bias_vy').value),
        )
        self.max_xy = gp('max_cmd_xy').value
        self.max_z = gp('max_cmd_z').value
        self.max_yaw = gp('max_cmd_yaw').value
        self.gate = CommandGate(geofence, gp('cmd_timeout').value)
        self.geofence_use_vision = gp('geofence_use_vision').value
        self.use_altitude = gp('use_altitude_topic').value
        self.altitude_timeout = gp('altitude_timeout').value
        self.use_vision = gp('use_vision').value
        self.vision_shadow = gp('vision_shadow').value
        # Visión al mando: la pose corregida alimenta control y geocerca
        self.vision_active = self.use_vision and not self.vision_shadow
        self.fusion = vision_fusion_from_params(self)
        self.vision_stats = {}

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
        self.create_subscription(Float64, '/bebop/altitude', self.altitude_cb, 10)

        # ── Visión ────────────────────────────────────────────
        if self.use_vision:
            self.create_subscription(
                PoseWithCovarianceStamped, '/vision/onboard/pose', self.vision_cb, 10)
            self.pub_fused = self.create_publisher(PoseStamped, '/vision/fused_pose', 10)
            self.pub_odom_raw = self.create_publisher(PoseStamped, '/vision/odom_pose', 10)
            self.pub_lock = self.create_publisher(Bool, '/vision/lock', 10)
            self.pub_vreset = self.create_publisher(Empty, '/vision/reset', 10)
            self.create_timer(0.1, self.lock_loop)

        # ── Estado ────────────────────────────────────────────
        self.current_local_pose = None
        self.fence_pose = None      # pose y velocidad (marco corral) para la geocerca
        self.fence_vel = None
        self.last_odom_time = None
        self.altitude = None
        self.last_altitude_time = None
        self.z_is_measured = False
        self._last_geofence_log = 0.0

        self.create_timer(0.05, self.watchdog_loop)   # 20 Hz
        self.create_timer(5.0, self.status_loop)
        self.get_logger().info(
            'Real Drone Bridge listo. /bridge/* → /bebop/*  |  watchdog 20 Hz')

    # ── Comandos discretos ────────────────────────────────────
    def takeoff_cb(self, _):
        # Re-anclar el marco de despegue al punto actual. Con visión al mando
        # no: el offset visual se calculó sobre el marco viejo.
        if not self.vision_active:
            self.takeoff_frame.origin = None
        self.fusion.takeoff(reanchor=not self.vision_active)
        self.gate.takeoff()
        self.pub_takeoff.publish(Empty())
        self.get_logger().info('🛫 TAKEOFF → /bebop/takeoff')

    def land_cb(self, _):
        self.gate.land()
        self.pub_land.publish(Empty())
        self.get_logger().info('🛬 LAND → /bebop/land')

    def emergency_cb(self, _):
        self.gate.land()
        self.pub_reset.publish(Empty())
        self.get_logger().error('🚨 EMERGENCY (corte de motores) → /bebop/reset')

    def flattrim_cb(self, _):
        self.pub_flattrim.publish(Empty())
        self.get_logger().info('📐 FLATTRIM → /bebop/flattrim')

    def cmd_vel_cb(self, msg):
        self.gate.command((
            clamp(msg.linear.x + self.trim_pitch, -self.max_xy, self.max_xy),
            clamp(msg.linear.y + self.trim_roll, -self.max_xy, self.max_xy),
            clamp(msg.linear.z, -self.max_z, self.max_z),
            clamp(msg.angular.z, -self.max_yaw, self.max_yaw)), time.time())

    # ── Odometría ─────────────────────────────────────────────
    def altitude_cb(self, msg):
        self.altitude = msg.data
        self.last_altitude_time = time.time()

    def odom_cb(self, msg):
        self.last_odom_time = time.time()
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        lx, ly, lz, lyaw = self.takeoff_frame.pose_to_local(p.x, p.y, p.z, yaw, t)
        if self.use_altitude:
            age = (None if self.last_altitude_time is None
                   else time.time() - self.last_altitude_time)
            lz, self.z_is_measured = select_altitude(
                lz, self.altitude, age, self.altitude_timeout)
        fence = (lx, ly, lz, lyaw)
        if self.use_vision:
            fused = self.fusion.odom(t, lx, ly, lyaw)
            if fused is not None:
                fx, fy, fyaw = fused
                if self.vision_active:
                    self._publish_stamped(self.pub_odom_raw, msg.header, lx, ly, lz, lyaw)
                    lx, ly, lyaw = fx, fy, fyaw
                else:
                    self._publish_stamped(self.pub_fused, msg.header, fx, fy, lz, fyaw)
                if self.vision_active or (self.geofence_use_vision and self.fusion.locked):
                    fence = (fx, fy, lz, fyaw)
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
        tw = msg.twist.twist.linear
        vx, vy, _ = self.takeoff_frame.body_vel_to_local(tw.x, tw.y, tw.z, fence[3])
        self.fence_pose, self.fence_vel = fence, (vx, vy)

    # ── Visión ────────────────────────────────────────────────
    def vision_cb(self, msg):
        z = self.current_local_pose[2] if self.z_is_measured and self.current_local_pose else None
        res = vision_update(self.fusion, msg, self.vision_stats, z)
        if res == 'init':
            self.pub_vreset.publish(Empty())
            dx, dy, dyaw = self.fusion.filter.offset
            self.get_logger().warn(
                f'Visión {res}: offset ({dx:+.2f}, {dy:+.2f}) m, {math.degrees(dyaw):+.1f}°')

    def lock_loop(self):
        self.pub_lock.publish(Bool(data=self.fusion.locked))

    def _publish_stamped(self, pub, header, x, y, z, yaw):
        msg = PoseStamped()
        msg.header = header
        msg.header.frame_id = 'corral'
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = x, y, z
        (msg.pose.orientation.x, msg.pose.orientation.y,
         msg.pose.orientation.z, msg.pose.orientation.w) = quaternion_from_yaw(yaw)
        pub.publish(msg)

    # ── Watchdog ──────────────────────────────────────────────
    def watchdog_loop(self):
        # Sin comando reciente: ceros (hover nativo). La geocerca actúa igual en el aire.
        now = time.time()
        (pitch, roll, gaz, yaw_rate), events = self.gate.output(
            now, self.fence_pose, self.fence_vel, self.z_is_measured)
        if events and now - self._last_geofence_log > 1.0:
            self._last_geofence_log = now
            self.get_logger().warn(f'Geocerca: {events}')
        cmd = Twist()
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
        if self.use_altitude and not self.z_is_measured:
            self.get_logger().warn(
                'Sin /bebop/altitude reciente: la z es la vz integrada y deriva. '
                '¿El driver tiene el parche de altura? (patches/)')
        if self.use_vision:
            self.get_logger().info(vision_status(self.fusion, self.vision_stats, self.vision_shadow))


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
