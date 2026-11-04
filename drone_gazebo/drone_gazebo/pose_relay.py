#!/usr/bin/env python3
"""
Pose exacta del Matrice en el marco de despegue de control_bebop.

Gazebo publica la pose en el marco del mundo (/matrice_100/pose_world). Los
nodos de control_bebop esperan el origen en el punto de despegue, x al frente,
y a la izquierda, z hacia arriba (/bebop1/pose_gt, que lee sim_odometry).
"""

import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node


class PoseRelay(Node):
    def __init__(self):
        super().__init__('pose_relay')
        # Reposo del Matrice sobre la plataforma (spawn en 1.92, -10, yaw 0)
        self.declare_parameter('origin_x', 1.92)
        self.declare_parameter('origin_y', -10.0)
        self.declare_parameter('origin_z', 0.415)
        self.origin = [self.get_parameter(n).value
                       for n in ('origin_x', 'origin_y', 'origin_z')]
        self.pub = self.create_publisher(Pose, '/bebop1/pose_gt', 10)
        self.create_subscription(Pose, '/matrice_100/pose_world', self.cb, 10)

    def cb(self, msg):
        # shortcut: solo traslada, supone yaw 0 al aparecer; si se cambia el yaw del spawn, rotar aquí
        msg.position.x -= self.origin[0]
        msg.position.y -= self.origin[1]
        msg.position.z -= self.origin[2]
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PoseRelay()
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
