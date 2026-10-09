#!/usr/bin/env python3
"""
GUI interactiva para visualizar la odometría del Bebop 2 en tiempo real.

Muestra:
  - Posición local (X, Y, Z) en metros (marco de despegue y calibrado)
  - Distancia euclidiana al origen
  - Orientación (Roll, Pitch, Yaw) en grados
  - Velocidades lineales (Vx, Vy, Vz)
  - Estado de vuelo actual (IDLE, TAKING_OFF, HOVER, MANUAL, etc.)
  - Registro de trayectoria con botón de Reset

Uso:
  ros2 run control_bebop gui_odometria
"""

import sys
import math
import threading
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from std_msgs.msg import Int32


STATE_NAMES = {
    0: '0 - IDLE',
    1: '1 - AUTOMATIC',
    2: '2 - TAKING_OFF',
    3: '3 - LANDING',
    4: '4 - EMERGENCY',
    5: '5 - HOVER',
    6: '6 - MANUAL',
    7: '7 - RECOVERY',
    8: '8 - RETURN_HOME',
}


def euler_from_quaternion(x, y, z, w):
    """Calcula Roll, Pitch, Yaw (rad) desde un cuaternión."""
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    t2 = max(-1.0, min(+1.0, +2.0 * (w * y - z * x)))
    pitch = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return roll, pitch, yaw


class OdomGuiNode(Node):
    def __init__(self, update_callback):
        super().__init__('gui_odometria_node')
        self.update_callback = update_callback

        # Suscribir a la pose local transformada / calibrada
        self.create_subscription(Pose, '/bebop1/pose', self.pose_cb, 10)
        # Suscribir a la odometría para velocidades
        self.create_subscription(Odometry, '/odom_local', self.odom_local_cb, 10)
        # Backup: odometría directa del driver
        self.create_subscription(Odometry, '/bebop/odom', self.bebop_odom_cb, 10)
        # Estado de vuelo
        self.create_subscription(Int32, '/flight_state', self.state_cb, 10)

        self.data = {
            'x': 0.0, 'y': 0.0, 'z': 0.0,
            'roll_deg': 0.0, 'pitch_deg': 0.0, 'yaw_deg': 0.0,
            'vx': 0.0, 'vy': 0.0, 'vz': 0.0,
            'state': 'Desconectado',
            'updates_count': 0
        }

    def pose_cb(self, msg: Pose):
        self.data['x'] = msg.position.x
        self.data['y'] = msg.position.y
        self.data['z'] = msg.position.z

        r, p, y = euler_from_quaternion(
            msg.orientation.x, msg.orientation.y,
            msg.orientation.z, msg.orientation.w
        )
        self.data['roll_deg'] = math.degrees(r)
        self.data['pitch_deg'] = math.degrees(p)
        self.data['yaw_deg'] = math.degrees(y)
        self.data['updates_count'] += 1
        self.update_callback(self.data)

    def odom_local_cb(self, msg: Odometry):
        self.data['vx'] = msg.twist.twist.linear.x
        self.data['vy'] = msg.twist.twist.linear.y
        self.data['vz'] = msg.twist.twist.linear.z

    def bebop_odom_cb(self, msg: Odometry):
        # Solo si /bebop1/pose no ha enviado nada aún
        if self.data['updates_count'] == 0:
            self.data['x'] = msg.pose.pose.position.x
            self.data['y'] = msg.pose.pose.position.y
            self.data['z'] = msg.pose.pose.position.z
            r, p, y = euler_from_quaternion(
                msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z, msg.pose.pose.orientation.w
            )
            self.data['yaw_deg'] = math.degrees(y)
            self.update_callback(self.data)

    def state_cb(self, msg: Int32):
        self.data['state'] = STATE_NAMES.get(msg.data, f'Desconocido ({msg.data})')
        self.update_callback(self.data)


class OdometriaApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Telemetría Bebop 2 - Odometría en Tiempo Real")
        self.root.geometry("480x560")
        self.root.minsize(420, 500)
        self.root.configure(bg="#1e1e2e")

        self.latest_data = {}
        self.min_max = {'max_dist': 0.0, 'max_z': 0.0}

        self._setup_style()
        self._build_ui()

    def _setup_style(self):
        self.style = ttk.Style()
        self.style.theme_use('clam')
        self.style.configure(".", background="#1e1e2e", foreground="#cdd6f4")
        self.style.configure("TLabelframe", background="#1e1e2e", foreground="#89b4fa")
        self.style.configure("TLabelframe.Label", background="#1e1e2e", foreground="#89b4fa", font=("Ubuntu", 11, "bold"))
        self.style.configure("TLabel", background="#1e1e2e", foreground="#cdd6f4", font=("Ubuntu", 10))

    def _build_ui(self):
        # Título y Estado
        header_frame = tk.Frame(self.root, bg="#181825", pady=10)
        header_frame.pack(fill="x", padx=10, pady=5)

        title_lbl = tk.Label(header_frame, text="🛸 Bebop 2 - Odometría Local",
                             font=("Ubuntu", 14, "bold"), bg="#181825", fg="#89b4fa")
        title_lbl.pack()

        self.status_lbl = tk.Label(header_frame, text="Estado: Esperando datos...",
                                   font=("Ubuntu", 10, "bold"), bg="#181825", fg="#f9e2af")
        self.status_lbl.pack(pady=2)

        # 1. Posición
        pos_frame = ttk.LabelFrame(self.root, text=" 📍 Posición Local (Marco Despegue) ")
        pos_frame.pack(fill="x", padx=12, pady=5)

        self.lbl_x = tk.Label(pos_frame, text="X (Frente):   +0.000 m", font=("Monospace", 13, "bold"),
                              bg="#1e1e2e", fg="#a6e3a1", anchor="w")
        self.lbl_x.pack(fill="x", padx=15, pady=3)

        self.lbl_y = tk.Label(pos_frame, text="Y (Izquierda): +0.000 m", font=("Monospace", 13, "bold"),
                              bg="#1e1e2e", fg="#a6e3a1", anchor="w")
        self.lbl_y.pack(fill="x", padx=15, pady=3)

        self.lbl_z = tk.Label(pos_frame, text="Z (Altura):    +0.000 m", font=("Monospace", 13, "bold"),
                              bg="#1e1e2e", fg="#89dceb", anchor="w")
        self.lbl_z.pack(fill="x", padx=15, pady=3)

        self.lbl_dist = tk.Label(pos_frame, text="Distancia 2D:   0.000 m", font=("Monospace", 11),
                                 bg="#1e1e2e", fg="#fab387", anchor="w")
        self.lbl_dist.pack(fill="x", padx=15, pady=2)

        # 2. Orientación
        ori_frame = ttk.LabelFrame(self.root, text=" 🧭 Orientación (Ángulos Euler) ")
        ori_frame.pack(fill="x", padx=12, pady=5)

        self.lbl_yaw = tk.Label(ori_frame, text="Yaw (Rumbo):   +0.00°", font=("Monospace", 11),
                                bg="#1e1e2e", fg="#cba6f7", anchor="w")
        self.lbl_yaw.pack(fill="x", padx=15, pady=2)

        self.lbl_pitch_roll = tk.Label(ori_frame, text="Pitch: +0.0°  |  Roll: +0.0°", font=("Monospace", 10),
                                       bg="#1e1e2e", fg="#bac2de", anchor="w")
        self.lbl_pitch_roll.pack(fill="x", padx=15, pady=2)

        # 3. Velocidades
        vel_frame = ttk.LabelFrame(self.root, text=" ⚡ Velocidades Lineales (m/s) ")
        vel_frame.pack(fill="x", padx=12, pady=5)

        self.lbl_vel = tk.Label(vel_frame, text="Vx: +0.00 m/s  |  Vy: +0.00 m/s  |  Vz: +0.00 m/s",
                                font=("Monospace", 10), bg="#1e1e2e", fg="#f5c2e7", anchor="w")
        self.lbl_vel.pack(fill="x", padx=15, pady=4)

        # Registro / Historial breve
        log_frame = ttk.LabelFrame(self.root, text=" 📋 Resumen ")
        log_frame.pack(fill="both", expand=True, padx=12, pady=5)

        self.lbl_max = tk.Label(log_frame, text="Dist. Máx recorrida: 0.00 m  |  Altura Máx: 0.00 m",
                                font=("Ubuntu", 9), bg="#1e1e2e", fg="#a6adc8")
        self.lbl_max.pack(pady=4)

        btn_reset = tk.Button(log_frame, text="Reiniciar Máximos", command=self.reset_max,
                              bg="#313244", fg="#cdd6f4", font=("Ubuntu", 9), relief="flat")
        btn_reset.pack(pady=2)

    def reset_max(self):
        self.min_max['max_dist'] = 0.0
        self.min_max['max_z'] = 0.0
        self.lbl_max.config(text="Dist. Máx recorrida: 0.00 m  |  Altura Máx: 0.00 m")

    def queue_update(self, data):
        self.latest_data = data
        # Ejecutar en hilo de Tkinter
        self.root.after_idle(self._apply_update)

    def _apply_update(self):
        d = self.latest_data
        if not d:
            return

        x = d.get('x', 0.0)
        y = d.get('y', 0.0)
        z = d.get('z', 0.0)
        dist_2d = math.hypot(x, y)

        if dist_2d > self.min_max['max_dist']:
            self.min_max['max_dist'] = dist_2d
        if z > self.min_max['max_z']:
            self.min_max['max_z'] = z

        self.status_lbl.config(text=f"Estado de Vuelo: {d.get('state', 'Conectado')}")

        self.lbl_x.config(text=f"X (Frente):   {x:+.3f} m")
        self.lbl_y.config(text=f"Y (Izquierda):{y:+.3f} m")
        self.lbl_z.config(text=f"Z (Altura):   {z:+.3f} m")
        self.lbl_dist.config(text=f"Distancia 2D:  {dist_2d:.3f} m")

        yaw = d.get('yaw_deg', 0.0)
        pitch = d.get('pitch_deg', 0.0)
        roll = d.get('roll_deg', 0.0)
        self.lbl_yaw.config(text=f"Yaw (Rumbo):  {yaw:+.2f}°")
        self.lbl_pitch_roll.config(text=f"Pitch: {pitch:+.1f}°  |  Roll: {roll:+.1f}°")

        vx = d.get('vx', 0.0)
        vy = d.get('vy', 0.0)
        vz = d.get('vz', 0.0)
        self.lbl_vel.config(text=f"Vx: {vx:+.2f} m/s  |  Vy: {vy:+.2f} m/s  |  Vz: {vz:+.2f} m/s")

        self.lbl_max.config(
            text=f"Dist. Máx recorrida: {self.min_max['max_dist']:.2f} m  |  Altura Máx: {self.min_max['max_z']:.2f} m"
        )


def main(args=None):
    rclpy.init(args=args)

    root = tk.Tk()
    app = OdometriaApp(root)

    node = OdomGuiNode(app.queue_update)

    ros_thread = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    ros_thread.start()

    try:
        root.mainloop()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

