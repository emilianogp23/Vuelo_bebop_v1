"""
Control PID para Bebop 2.

Suscribe:
    /bebop1/pose  (geometry_msgs/Pose)   ← Posición actual (Gazebo)
    /goal         (geometry_msgs/Pose)   ← Setpoint de posición
    /flight_state (std_msgs/Int32)       ← Estado del sistema

Publica:
    /bebop1/cmd_vel (geometry_msgs/Twist) ← Velocidades al dron
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import Int32


# ── Estados (deben coincidir con flight_manager.py) ───────────
IDLE = 0
AUTOMATIC = 1
TAKING_OFF = 2
LANDING = 3
EMERGENCY = 4
HOVER = 5
MANUAL = 6
RECOVERY = 7
RETURN_HOME = 8

# Estados en los que el PID debe seguir el goal
ACTIVE_STATES = {AUTOMATIC, TAKING_OFF, LANDING, HOVER, MANUAL,
                 RECOVERY, RETURN_HOME}


def clamp(value, min_value, max_value):
    return max(min_value, min(value, max_value))


def normalizar_angulos(angulo):
    while angulo > math.pi:
        angulo -= 2 * math.pi
    while angulo < -math.pi:
        angulo += 2 * math.pi
    return angulo


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


class PID():
    def __init__(self):
        self.k = 0.0
        # kp
        self.kp = 0.2
        self.kp_roll = 0.4
        self.kp_pitch = 0.4
        self.kp_yaw = 0.4
        # Kd
        self.kd = 0.05
        self.kd_roll = 0.08
        self.kd_pitch = 0.08
        self.kd_yaw = 0.08
        # ki
        self.ki = 0.0
        self.ki_roll = 0.0
        self.ki_pitch = 0.0
        self.ki_yaw = 0.0

        self.error_altitud = 0.0
        self.error_pitch = 0.0
        self.error_roll = 0.0
        self.error_yaw = 0.0

        self.integrador_altitud = 0.0
        self.integrador_roll = 0.0
        self.integrador_pitch = 0.0

    def reset_integrators(self):
        """Reinicia acumuladores integrales para evitar arrastres entre estados."""
        self.integrador_altitud = 0.0
        self.integrador_roll = 0.0
        self.integrador_pitch = 0.0
        self.error_altitud = 0.0
        self.error_pitch = 0.0
        self.error_roll = 0.0
        self.error_yaw = 0.0

    def control_altitud(self, altitud_des, altitud_act, dt):
        error_actual = altitud_des - altitud_act
        error_derivado = (error_actual - self.error_altitud) / max(dt, 1e-4)
        self.integrador_altitud = clamp(
            self.integrador_altitud + error_actual * dt, -0.5, 0.5)
        comando_z = (
            (clamp(error_actual, -1, 1) * self.kp)
            + (error_derivado * self.kd)
            + (self.integrador_altitud * self.ki)
            + self.k
        )
        self.error_altitud = error_actual
        return comando_z

    def control_roll(self, roll_obj, roll_act, dt):
        error_actual = roll_obj - roll_act
        error_derivado = (error_actual - self.error_roll) / max(dt, 1e-4)
        self.integrador_roll = clamp(
            self.integrador_roll + error_actual * dt, -0.3, 0.3)
        comando_y = (
            clamp(error_actual, -1, 1) * self.kp_roll
            + error_derivado * self.kd_roll
            + self.integrador_roll * self.ki_roll
        )
        self.error_roll = error_actual
        return comando_y

    def control_pitch(self, pitch_obj, pitch_act, dt):
        error_actual = pitch_obj - pitch_act
        error_derivado = (error_actual - self.error_pitch) / max(dt, 1e-4)
        self.integrador_pitch = clamp(
            self.integrador_pitch + error_actual * dt, -0.3, 0.3)
        comando_x = (
            clamp(error_actual, -1, 1) * self.kp_pitch
            + error_derivado * self.kd_pitch
            + self.integrador_pitch * self.ki_pitch
        )
        self.error_pitch = error_actual
        return comando_x

    def control_yaw(self, yaw_obj, yaw_act, dt, wz):
        error_actual = normalizar_angulos(yaw_obj - yaw_act)
        # Usar gyro/derivada (wz) para amortiguamiento
        error_derivado = -wz
        comando_yaw = (
            clamp(error_actual, -1, 1) * self.kp_yaw
            + error_derivado * self.kd_yaw
        )
        comando_yaw = clamp(comando_yaw, -1, 1)
        self.error_yaw = error_actual
        return comando_yaw


class DroneController(Node):
    def __init__(self):
        super().__init__('drone_controller')
        self.pid = PID()

        # ── Parámetros configurables ──
        self.declare_parameter('cmd_vel_topic', '/bebop1/cmd_vel_raw')
        self.declare_parameter('use_real_drone', False)
        self.declare_parameter('pid_in_hover', False)
        self.declare_parameter('takeoff_height', 1.0)
        self.declare_parameter('arena_limit', 2.0)
        self.declare_parameter('max_vel_xy', 0.25)
        self.declare_parameter('max_vel_z', 0.40)
        self.declare_parameter('max_vel_yaw', 0.40)

        # Ganancias PID configurables
        self.declare_parameter('kp_xy', 0.35)
        self.declare_parameter('kd_xy', 0.08)
        self.declare_parameter('ki_xy', 0.02)
        self.declare_parameter('kp_z', 0.40)
        self.declare_parameter('kd_z', 0.08)
        self.declare_parameter('ki_z', 0.02)
        self.declare_parameter('kp_yaw', 0.40)
        self.declare_parameter('kd_yaw', 0.08)
        self.declare_parameter('ki_yaw', 0.0)

        # Asignar ganancias a la clase PID
        self.pid.kp_pitch = self.get_parameter('kp_xy').value
        self.pid.kp_roll = self.get_parameter('kp_xy').value
        self.pid.kd_pitch = self.get_parameter('kd_xy').value
        self.pid.kd_roll = self.get_parameter('kd_xy').value
        self.pid.ki_pitch = self.get_parameter('ki_xy').value
        self.pid.ki_roll = self.get_parameter('ki_xy').value

        self.pid.kp = self.get_parameter('kp_z').value
        self.pid.kd = self.get_parameter('kd_z').value
        self.pid.ki = self.get_parameter('ki_z').value

        self.pid.kp_yaw = self.get_parameter('kp_yaw').value
        self.pid.kd_yaw = self.get_parameter('kd_yaw').value
        self.pid.ki_yaw = self.get_parameter('ki_yaw').value

        # Publicadores
        topic_name = self.get_parameter('cmd_vel_topic').value
        self.pub_vel = self.create_publisher(Twist, topic_name, 10)

        # Suscriptores
        self.sub_odom = self.create_subscription(
            Pose, '/bebop1/pose', self.odom_callback, 10)
        self.sub_goal = self.create_subscription(
            Pose, '/goal', self.goal_callback, 10)
        self.sub_state = self.create_subscription(
            Int32, '/flight_state', self.state_callback, 10)
        self.sub_joy = self.create_subscription(
            Twist, '/joy_cmd', self.joy_callback, 10)

        # Variables de estado
        self.x_actual = 0.0
        self.y_actual = 0.0
        self.z_actual = 0.0
        self.yaw_actual = 0.0
        self.yaw_rate = 0.0  # Estimación de velocidad angular yaw

        self.setpoint_x = 0.0
        self.setpoint_y = 0.0
        self.setpoint_z = 0.0
        self.setpoint_yaw = 0.0
        
        self.manual_joy = Twist()

        # Máquina de estados y timer
        self.estado = IDLE
        self.dt_nominal = 0.1
        self.last_loop_time = self.get_clock().now()
        self.timer = self.create_timer(self.dt_nominal, self.control_loop)

        self.get_logger().info(f'⚙️ Control PID iniciado — Publicando en: {topic_name}')

    def joy_callback(self, msg):
        self.manual_joy = msg

    def odom_callback(self, msg):
        self.x_actual = msg.position.x
        self.y_actual = msg.position.y
        self.z_actual = msg.position.z

        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w

        _, _, yaw = euler_from_quaternion(qx, qy, qz, qw)

        # Estimar yaw rate por diferencias finitas
        if self.dt_nominal > 0:
            self.yaw_rate = normalizar_angulos(
                yaw - self.yaw_actual) / self.dt_nominal

        self.yaw_actual = yaw

    def goal_callback(self, msg):
        """Actualiza setpoints — solo si estamos en un estado activo."""
        if self.estado not in ACTIVE_STATES:
            return

        self.setpoint_x = msg.position.x
        self.setpoint_y = msg.position.y
        self.setpoint_z = msg.position.z

        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w

        _, _, yaw = euler_from_quaternion(qx, qy, qz, qw)
        self.setpoint_yaw = yaw

    def state_callback(self, msg):
        prev = self.estado
        self.estado = msg.data

        if self.estado != prev:
            self.pid.reset_integrators()

        if self.estado == TAKING_OFF and prev != TAKING_OFF:
            takeoff_h = self.get_parameter('takeoff_height').value
            self.get_logger().info(
                f'Despegue: objetivo {takeoff_h}m')
            self.setpoint_x = self.x_actual
            self.setpoint_y = self.y_actual
            self.setpoint_z = takeoff_h
            self.setpoint_yaw = self.yaw_actual

    def control_loop(self):
        lim_x = self.get_parameter('arena_limit').value
        lim_y = lim_x

        now = self.get_clock().now()
        dt = (now - self.last_loop_time).nanoseconds * 1e-9
        self.last_loop_time = now
        if dt <= 0.005 or dt > 0.5:
            dt = self.dt_nominal

        # Advertencia de límite
        if (abs(self.x_actual) > lim_x or abs(self.y_actual) > lim_y):
            self.get_logger().warn('¡Límite de arena!')

        twist = Twist()

        if self.estado == IDLE:
            self.pub_vel.publish(twist)
            return

        elif self.estado == EMERGENCY:
            self.pub_vel.publish(twist)
            return

        elif self.estado == MANUAL:
            self.pub_vel.publish(self.manual_joy)
            return

        # Dron real: despegue, aterrizaje y hover los hace el autopiloto
        # nativo del Bebop (comando cero = mantener posición con flujo óptico)
        if (self.get_parameter('use_real_drone').value
                and self.estado in (TAKING_OFF, LANDING, HOVER)
                and not self.get_parameter('pid_in_hover').value):
            self.pub_vel.publish(twist)
            return

        # Para todos los estados activos (AUTOMATIC, TAKING_OFF,
        # LANDING, HOVER, MANUAL, RECOVERY, RETURN_HOME):
        # El PID sigue el setpoint actual que viene de /goal

        # Control PID
        comando_z = self.pid.control_altitud(
            self.setpoint_z, self.z_actual, dt)
        comando_roll = self.pid.control_roll(
            self.setpoint_y, self.y_actual, dt)
        comando_pitch = self.pid.control_pitch(
            self.setpoint_x, self.x_actual, dt)
        comando_yaw = self.pid.control_yaw(
            self.setpoint_yaw, self.yaw_actual, dt, self.yaw_rate)

        # Transformación mundo → dron (rotar por yaw actual)
        cos_yaw = math.cos(self.yaw_actual)
        sin_yaw = math.sin(self.yaw_actual)
        comando_p = comando_pitch * cos_yaw + comando_roll * sin_yaw
        comando_r = -comando_pitch * sin_yaw + comando_roll * cos_yaw

        # Publicar velocidades
        lim_velxy = self.get_parameter('max_vel_xy').value
        lim_vel_z = self.get_parameter('max_vel_z').value
        lim_vel_yaw = self.get_parameter('max_vel_yaw').value

        twist.linear.x = clamp(comando_p, -lim_velxy, lim_velxy)
        twist.linear.y = clamp(comando_r, -lim_velxy, lim_velxy)
        twist.linear.z = clamp(comando_z, -lim_vel_z, lim_vel_z)
        twist.angular.z = clamp(comando_yaw, -lim_vel_yaw, lim_vel_yaw)

        self.pub_vel.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    drone_controller = DroneController()
    try:
        rclpy.spin(drone_controller)
    except KeyboardInterrupt:
        pass
    finally:
        drone_controller.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()