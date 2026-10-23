#!/usr/bin/env python3
"""
Odometría simulada del Bebop: deriva como la real y se corrige con ArUco.

En Gazebo la pose es exacta. Este nodo la convierte en una odometría con los
errores del Bebop real (escala, sesgo de velocidad y deriva de yaw, solo en el
aire) y la fusiona con /vision/onboard/pose igual que real_drone_bridge
(VisionFusion de vision.py). Así se prueba la corrección visual sin volar.

Además hace de bridge para los comandos, como real_drone_bridge: recibe
/bridge/cmd_vel_raw, /bridge/takeoff, /bridge/land y /bridge/emergency, aplica la
misma geocerca (CommandGate de safety.py, parámetros de real_drone.yaml) y el
watchdog, y publica /bebop1/cmd_vel a 20 Hz.

Entradas:
    /bebop1/pose_gt       (Pose) pose exacta de Gazebo
    /vision/onboard/pose  (PoseWithCovarianceStamped) de aruco_localizer
    /bridge/cmd_vel_raw   (Twist) de control_PID
Salidas:
    /bebop1/pose          lo que usa el control: corregida (use_vision y no
                          vision_shadow) u odometría con deriva
    /vision/fused_pose    corregida, en modo sombra
    /vision/odom_pose     odometría con deriva sin corregir, con la visión al mando
    /vision/lock, /vision/reset   como real_drone_bridge
La z se pasa exacta (en el real viene de /bebop/altitude).
"""
import math

import rclpy
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped, Twist
from rclpy.node import Node
from std_msgs.msg import Bool, Empty

from control_bebop.frames import quaternion_from_yaw, rotate2d, yaw_from_quaternion
from control_bebop.real_drone_bridge import (geofence_params_from_node,
                                             vision_fusion_from_params, vision_status,
                                             vision_update)
from control_bebop.safety import CommandGate


class SimOdometry(Node):
    def __init__(self):
        super().__init__('sim_odometry')
        # El real lee ~1.24× la distancia (scale_x 0.807 en real_drone_calibrated.yaml)
        self.declare_parameter('odom_scale', 1.15)
        self.declare_parameter('odom_bias_vx', 0.010)       # m/s en el aire
        self.declare_parameter('odom_bias_vy', -0.008)
        self.declare_parameter('odom_yaw_drift_deg_s', 0.05)
        self.declare_parameter('airborne_z', 0.10)   # en reposo el modelo queda en z 0.051
        self.declare_parameter('use_vision', True)
        self.declare_parameter('vision_shadow', False)
        gp = lambda n: self.get_parameter(n).value  # noqa: E731
        self.scale = gp('odom_scale')
        self.bias = (gp('odom_bias_vx'), gp('odom_bias_vy'))
        self.yaw_drift = math.radians(gp('odom_yaw_drift_deg_s'))
        self.airborne_z = gp('airborne_z')
        self.use_vision = gp('use_vision')
        self.shadow = gp('vision_shadow')
        self.fusion = vision_fusion_from_params(self)
        self.stats = {}
        self.gate = CommandGate(geofence_params_from_node(self))
        self.geofence_use_vision = gp('geofence_use_vision')
        self.fence_pose = None
        self.fence_vel = None
        self.prev_fence = None    # (t, x, y) para la velocidad
        self._last_fence_log = 0.0

        self.prev = None          # (t, x, y) exactos
        self.odom = None          # (x, y) con deriva
        self.yaw_err = 0.0
        self.took_off = False
        self.z = None             # altura exacta (en el real: /bebop/altitude)

        self.pub_pose = self.create_publisher(Pose, '/bebop1/pose', 10)
        self.pub_cmd = self.create_publisher(Twist, '/bebop1/cmd_vel', 10)
        self.create_subscription(Twist, '/bridge/cmd_vel_raw', self.cmd_cb, 10)
        self.create_subscription(Empty, '/bridge/takeoff', lambda _: self.gate.takeoff(), 10)
        self.create_subscription(Empty, '/bridge/land', lambda _: self.gate.land(), 10)
        self.create_subscription(Empty, '/bridge/emergency', lambda _: self.gate.land(), 10)
        self.create_timer(0.05, self.cmd_loop)
        self.create_subscription(Pose, '/bebop1/pose_gt', self.gt_cb, 10)
        if self.use_vision:
            self.create_subscription(PoseWithCovarianceStamped, '/vision/onboard/pose',
                                     self.vision_cb, 10)
            self.pub_fused = self.create_publisher(PoseStamped, '/vision/fused_pose', 10)
            self.pub_odom_raw = self.create_publisher(PoseStamped, '/vision/odom_pose', 10)
            self.pub_lock = self.create_publisher(Bool, '/vision/lock', 10)
            self.pub_vreset = self.create_publisher(Empty, '/vision/reset', 10)
            self.create_timer(0.1, lambda: self.pub_lock.publish(Bool(data=self.fusion.locked)))
            self.create_timer(5.0, self.status_cb)
        self.get_logger().info(
            f'sim_odometry: escala {self.scale}, sesgo {self.bias} m/s, '
            f'visión {"sombra" if self.shadow else "al mando" if self.use_vision else "apagada"}')

    def gt_cb(self, msg):
        t = self.get_clock().now().nanoseconds * 1e-9
        p, q = msg.position, msg.orientation
        yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        airborne = p.z > self.airborne_z
        if self.prev is None:
            self.odom = (p.x, p.y)   # arranca en la pose real: el origen del corral
        else:
            dt = max(0.0, t - self.prev[0])
            if airborne:
                self.yaw_err += self.yaw_drift * dt
            dx, dy = rotate2d(p.x - self.prev[1], p.y - self.prev[2], self.yaw_err)
            bx, by = (self.bias[0] * dt, self.bias[1] * dt) if airborne else (0.0, 0.0)
            self.odom = (self.odom[0] + self.scale * dx + bx, self.odom[1] + self.scale * dy + by)
        self.prev = (t, p.x, p.y)
        self.z = p.z
        x, y, oyaw = self.odom[0], self.odom[1], yaw + self.yaw_err
        fence = (x, y, oyaw)

        if self.use_vision:
            if airborne and not self.took_off:
                self.took_off = True
                self.fusion.takeoff(reanchor=False)
            fused = self.fusion.odom(t, x, y, oyaw)
            if fused is not None:
                if self.shadow:
                    self.pub_fused.publish(self._stamped(fused, p.z))
                else:
                    self.pub_odom_raw.publish(self._stamped((x, y, oyaw), p.z))
                    x, y, oyaw = fused
                if not self.shadow or (self.geofence_use_vision and self.fusion.locked):
                    fence = fused
        self.pub_pose.publish(self._pose(x, y, p.z, oyaw))
        self._update_fence(t, fence, p.z)

    def _update_fence(self, t, fence, z):
        """Pose y velocidad (diferencia en ≥ 0.1 s) para la geocerca."""
        fx, fy, fyaw = fence
        self.fence_pose = (fx, fy, z, fyaw)
        if self.prev_fence is None or t - self.prev_fence[0] >= 0.1:
            if self.prev_fence is not None:
                dt = t - self.prev_fence[0]
                self.fence_vel = ((fx - self.prev_fence[1]) / dt, (fy - self.prev_fence[2]) / dt)
            self.prev_fence = (t, fx, fy)

    def cmd_cb(self, msg):
        self.gate.command((msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z),
                          self.get_clock().now().nanoseconds * 1e-9)

    def cmd_loop(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        (vx, vy, vz, wz), events = self.gate.output(now, self.fence_pose, self.fence_vel)
        if events and now - self._last_fence_log > 1.0:
            self._last_fence_log = now
            self.get_logger().warn(f'Geocerca: {events}')
        cmd = Twist()
        cmd.linear.x, cmd.linear.y, cmd.linear.z, cmd.angular.z = vx, vy, vz, wz
        self.pub_cmd.publish(cmd)

    def vision_cb(self, msg):
        res = vision_update(self.fusion, msg, self.stats, self.z)
        if res == 'init':
            self.pub_vreset.publish(Empty())
            self.get_logger().warn(f'Visión {res}')

    def status_cb(self):
        self.get_logger().info(vision_status(self.fusion, self.stats, self.shadow))

    @staticmethod
    def _pose(x, y, z, yaw):
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = float(x), float(y), float(z)
        (pose.orientation.x, pose.orientation.y,
         pose.orientation.z, pose.orientation.w) = quaternion_from_yaw(yaw)
        return pose

    def _stamped(self, fused, z):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'corral'
        msg.pose = self._pose(fused[0], fused[1], z, fused[2])
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = SimOdometry()
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
