#!/usr/bin/env python3
"""
Previsualización de la trayectoria SIN Gazebo y SIN dron.

Corre el MISMO `trajectory_planner` del vuelo real, con los mismos parámetros
(real_drone.yaml), y un "dron virtual" que sigue el /goal con un lag de primer
orden y velocidad limitada. Dibuja en vivo: arena, círculo objetivo, setpoints
(/goal) y la trayectoria del dron virtual.

Uso:
    ros2 launch control_bebop preview_trajectory.launch.py
"""

import math
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Int32, Bool


class FakeDrone(Node):
    def __init__(self):
        super().__init__('trajectory_preview')
        self.declare_parameter('target_x', 1.0)
        self.declare_parameter('target_y', 0.0)
        self.declare_parameter('circle_radius', 0.40)
        self.declare_parameter('arena_half_size', 2.0)
        self.declare_parameter('circle_height', 1.0)
        self.declare_parameter('tau', 0.6)        # constante de tiempo (s)
        self.declare_parameter('max_speed', 0.5)  # m/s
        self.declare_parameter('start_delay', 2.0)

        g = lambda n: self.get_parameter(n).value
        self.cx, self.cy = g('target_x'), g('target_y')
        self.r, self.arena = g('circle_radius'), g('arena_half_size')
        self.tau, self.vmax = g('tau'), g('max_speed')
        self.delay = g('start_delay')

        self.x = self.y = 0.0
        self.z = g('circle_height')   # se asume ya despegado
        self.yaw = 0.0
        self.goal = None
        self.done = False
        self.t = 0.0
        self.path = [(0.0, 0.0)]
        self.goals = []
        self.started = False

        self.create_subscription(Pose, '/goal', self.goal_cb, 10)
        self.create_subscription(Bool, '/trajectory_done', self.done_cb, 10)
        self.pub_pose = self.create_publisher(Pose, '/bebop1/pose', 10)
        self.pub_state = self.create_publisher(Int32, '/flight_state', 10)
        self.dt = 0.02
        self.create_timer(self.dt, self.step)

    def goal_cb(self, m):
        self.goal = (m.position.x, m.position.y)
        self.goals.append(self.goal)
        qz, qw = m.orientation.z, m.orientation.w
        self.yaw = 2.0 * math.atan2(qz, qw)

    def done_cb(self, m):
        if m.data:
            self.done = True
            self.get_logger().info('Órbita completada en la previsualización.')

    def step(self):
        self.t += self.dt
        s = Int32()
        # AUTOMATIC tras start_delay, luego HOVER al terminar
        s.data = 5 if self.done else (1 if self.t > self.delay else 5)
        self.pub_state.publish(s)

        if self.goal is not None and not self.done:
            ex, ey = self.goal[0] - self.x, self.goal[1] - self.y
            vx, vy = ex / self.tau, ey / self.tau
            v = math.hypot(vx, vy)
            if v > self.vmax:
                vx, vy = vx / v * self.vmax, vy / v * self.vmax
            self.x += vx * self.dt
            self.y += vy * self.dt
            self.path.append((self.x, self.y))

        p = Pose()
        p.position.x, p.position.y, p.position.z = self.x, self.y, self.z
        p.orientation.z = math.sin(self.yaw / 2.0)
        p.orientation.w = math.cos(self.yaw / 2.0)
        self.pub_pose.publish(p)


def main(args=None):
    rclpy.init(args=args)
    node = FakeDrone()
    th = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    th.start()

    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle

    fig, ax = plt.subplots(figsize=(7, 7))
    a = node.arena
    ax.add_patch(Rectangle((-a, -a), 2 * a, 2 * a, fill=False, ec='red',
                           lw=2, label='Arena'))
    ang = [i * 2 * math.pi / 200 for i in range(201)]
    ax.plot([node.cx + node.r * math.cos(t) for t in ang],
            [node.cy + node.r * math.sin(t) for t in ang],
            'g--', label=f'Círculo objetivo (r={node.r:.2f} m)')
    ax.plot(0, 0, 'k^', ms=10, label='Despegue (0,0)')
    ax.plot(node.cx, node.cy, 'g+', ms=12)
    goals_l, = ax.plot([], [], 'b.', ms=2, label='Setpoints /goal')
    path_l, = ax.plot([], [], 'm-', lw=2, label='Dron virtual')
    dot, = ax.plot([], [], 'mo', ms=9)
    ax.set_xlim(-a - 0.2, a + 0.2)
    ax.set_ylim(-a - 0.2, a + 0.2)
    ax.set_aspect('equal')
    ax.grid(True)
    ax.set_xlabel('X (m, adelante)')
    ax.set_ylabel('Y (m)')
    ax.set_title('Previsualización de trayectoria (sin dron)')
    ax.legend(loc='upper left', fontsize=8)

    try:
        while plt.fignum_exists(fig.number) and rclpy.ok():
            if node.path:
                px, py = zip(*node.path[-5000:])
                path_l.set_data(px, py)
                dot.set_data([node.x], [node.y])
            if node.goals:
                gx, gy = zip(*node.goals[-5000:])
                goals_l.set_data(gx, gy)
            plt.pause(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

