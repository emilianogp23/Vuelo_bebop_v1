"""
Control PID para Bebop 2 con Feedforward y control adaptativo de Z.

Suscribe:
    /bebop1/pose  (geometry_msgs/Pose)   ← Posición actual (marco despegue o Gazebo)
    /goal         (geometry_msgs/Pose)   ← Setpoint de posición
    /goal_vel     (geometry_msgs/Twist)  ← Velocidad del setpoint (feedforward,
                                           marco de despegue; opcional)
    /flight_state (std_msgs/Int32)       ← Estado del sistema
    /joy_cmd      (geometry_msgs/Twist)  ← Comandos de joystick para modo manual

Publica:
    /bebop1/cmd_vel (geometry_msgs/Twist) ← Velocidades al dron (o al bridge)

Feedforward (dron real): el Bebop responde a pitch/roll con una velocidad
aproximadamente proporcional (medido en vuelos: ~0.8 m/s por unidad en
pitch, ~0.55 en roll, retardo ~0.6 s) y a yaw con ~0.21 rad/s por unidad.
cmd_ff = v_ref * ff_gain, en marco del cuerpo.

z_mode:
    native : gaz = 0 en vuelo automático mientras la altitud esté en zona segura.
             El Bebop mantiene su altura nativamente con su sonar y barómetro.
             Recomendado con ros2_bebop_driver, cuya z es velocidad vertical INTEGRADA
             (acumula deriva y hace que el PID baje y suba erróneamente).
    pid    : lazo cerrado continuo sobre la posición z de odometría.
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
        self.kp = 0.50
        self.kp_roll = 0.70
        self.kp_pitch = 0.70
        self.kp_yaw = 0.80
        # Kd
        self.kd = 0.10
        self.kd_roll = 0.15
        self.kd_pitch = 0.15
        self.kd_yaw = 0.12
        # ki
        self.ki = 0.02
        self.ki_roll = 0.03
        self.ki_pitch = 0.03
        self.ki_yaw = 0.0

        # Límites integrador
        self.int_limit_xy = 0.5
        self.int_limit_z = 0.5

        self.error_altitud = 0.0
        self.error_pitch = 0.0
        self.error_roll = 0.0
        self.error_yaw = 0.0

        self.integrador_altitud = 0.0
        self.integrador_roll = 0.0
        self.integrador_pitch = 0.0

        # Derivada sobre la MEDICIÓN (filtrada) para evitar "derivative kick"
        # cuando el setpoint salta, y picos por odometría a 15 Hz.
        self.d_alpha = 0.5          # Filtro pasa-bajas (0=sin filtro, →1 más suave)
        self._prev_meas = {}        # eje → última medición
        self._d_filt = {}           # eje → derivada filtrada

    def reset_integrators(self):
        """Reinicia acumuladores integrales para evitar arrastres entre estados."""
        self.integrador_altitud = 0.0
        self.integrador_roll = 0.0
        self.integrador_pitch = 0.0
        self.error_altitud = 0.0
        self.error_pitch = 0.0
        self.error_roll = 0.0
        self.error_yaw = 0.0
        self._prev_meas.clear()
        self._d_filt.clear()

    def _meas_derivative(self, axis, meas, dt):
        """-d(medición)/dt filtrada (equivale a d(error)/dt con setpoint fijo)."""
        prev = self._prev_meas.get(axis)
        self._prev_meas[axis] = meas
        if prev is None:
            self._d_filt[axis] = 0.0
            return 0.0
        raw = -(meas - prev) / max(dt, 1e-4)
        filt = self.d_alpha * self._d_filt.get(axis, 0.0) + (1.0 - self.d_alpha) * raw
        self._d_filt[axis] = filt
        return filt

    def control_altitud(self, altitud_des, altitud_act, dt):
        error_actual = altitud_des - altitud_act
        error_derivado = self._meas_derivative('z', altitud_act, dt)
        self.integrador_altitud = clamp(
            self.integrador_altitud + error_actual * dt, -self.int_limit_z, self.int_limit_z)
        comando_z = (
            (clamp(error_actual, -1.0, 1.0) * self.kp)
            + (error_derivado * self.kd)
            + (self.integrador_altitud * self.ki)
            + self.k
        )
        self.error_altitud = error_actual
        return comando_z

    def control_roll(self, roll_obj, roll_act, dt):
        error_actual = roll_obj - roll_act
        error_derivado = self._meas_derivative('y', roll_act, dt)
        self.integrador_roll = clamp(
            self.integrador_roll + error_actual * dt, -self.int_limit_xy, self.int_limit_xy)
        comando_y = (
            clamp(error_actual, -1.0, 1.0) * self.kp_roll
            + error_derivado * self.kd_roll
            + self.integrador_roll * self.ki_roll
        )
        self.error_roll = error_actual
        return comando_y

    def control_pitch(self, pitch_obj, pitch_act, dt):
        error_actual = pitch_obj - pitch_act
        error_derivado = self._meas_derivative('x', pitch_act, dt)
        self.integrador_pitch = clamp(
            self.integrador_pitch + error_actual * dt, -self.int_limit_xy, self.int_limit_xy)
        comando_x = (
            clamp(error_actual, -1.0, 1.0) * self.kp_pitch
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
            clamp(error_actual, -1.0, 1.0) * self.kp_yaw
            + error_derivado * self.kd_yaw
        )
        comando_yaw = clamp(comando_yaw, -1.0, 1.0)
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
        self.declare_parameter('max_vel_xy', 0.35)
        self.declare_parameter('max_vel_z', 0.40)
        self.declare_parameter('max_vel_yaw', 0.60)

        # Ganancias PID configurables
        self.declare_parameter('kp_xy', 0.70)
        self.declare_parameter('kd_xy', 0.15)
        self.declare_parameter('ki_xy', 0.03)
        self.declare_parameter('kp_z', 0.50)
        self.declare_parameter('kd_z', 0.10)
        self.declare_parameter('ki_z', 0.02)
        self.declare_parameter('kp_yaw', 0.80)
        self.declare_parameter('kd_yaw', 0.12)
        self.declare_parameter('ki_yaw', 0.0)

        # Límites integrador
        self.declare_parameter('int_limit_xy', 0.5)
        self.declare_parameter('int_limit_z', 0.5)

        # Feedforward de velocidad
        self.declare_parameter('enable_feedforward', True)
        self.declare_parameter('ff_gain_xy', 1.25)
        self.declare_parameter('ff_gain_yaw', 4.5)

        # Modo de altitud Z ('native' recomendado para dron real con ros2_bebop_driver)
        self.declare_parameter('z_mode', 'native')

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

        self.pid.int_limit_xy = self.get_parameter('int_limit_xy').value
        self.pid.int_limit_z = self.get_parameter('int_limit_z').value

        self.enable_ff = self.get_parameter('enable_feedforward').value
        self.ff_gain_xy = self.get_parameter('ff_gain_xy').value
        self.ff_gain_yaw = self.get_parameter('ff_gain_yaw').value
        self.z_mode = str(self.get_parameter('z_mode').value).lower()

        # Publicadores
        topic_name = self.get_parameter('cmd_vel_topic').value
        self.pub_vel = self.create_publisher(Twist, topic_name, 10)

        # Suscriptores
        self.sub_odom = self.create_subscription(
            Pose, '/bebop1/pose', self.odom_callback, 10)
        self.sub_goal = self.create_subscription(
            Pose, '/goal', self.goal_callback, 10)
        self.sub_goal_vel = self.create_subscription(
            Twist, '/goal_vel', self.goal_vel_callback, 10)
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
        self._last_odom_stamp = None

        self.setpoint_x = 0.0
        self.setpoint_y = 0.0
        self.setpoint_z = 0.0
        self.setpoint_yaw = 0.0

        # Feedforward
        self.goal_vx = 0.0
        self.goal_vy = 0.0
        self.goal_vz = 0.0
        self.goal_vyaw = 0.0
        self._last_goal_vel_time = self.get_clock().now()

        self.manual_joy = Twist()

        # Máquina de estados y timer
        self.estado = IDLE
        self.dt_nominal = 0.1
        self.last_loop_time = self.get_clock().now()
        self.timer = self.create_timer(self.dt_nominal, self.control_loop)

        self.get_logger().info(
            f'⚙️ Control PID iniciado — Publicando en: {topic_name} | '
            f'z_mode={self.z_mode} | feedforward={self.enable_ff}'
        )

    def joy_callback(self, msg):
        self.manual_joy = msg

    def goal_vel_callback(self, msg):
        self.goal_vx = msg.linear.x
        self.goal_vy = msg.linear.y
        self.goal_vz = msg.linear.z
        self.goal_vyaw = msg.angular.z
        self._last_goal_vel_time = self.get_clock().now()

    def odom_callback(self, msg):
        self.x_actual = msg.position.x
        self.y_actual = msg.position.y
        self.z_actual = msg.position.z

        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w

        _, _, yaw = euler_from_quaternion(qx, qy, qz, qw)

        # Estimar yaw rate con el tiempo REAL entre mensajes de odometría
        now = self.get_clock().now()
        if self._last_odom_stamp is not None:
            dt_odom = (now - self._last_odom_stamp).nanoseconds * 1e-9
            if 1e-3 < dt_odom < 0.5:
                self.yaw_rate = normalizar_angulos(yaw - self.yaw_actual) / dt_odom
        self._last_odom_stamp = now

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
            self.goal_vx = 0.0
            self.goal_vy = 0.0
            self.goal_vz = 0.0
            self.goal_vyaw = 0.0

        if self.estado == TAKING_OFF and prev != TAKING_OFF:
            takeoff_h = self.get_parameter('takeoff_height').value
            self.get_logger().info(f'Despegue: objetivo {takeoff_h}m')
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
        if abs(self.x_actual) > lim_x or abs(self.y_actual) > lim_y:
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

        # Expiración de feedforward si no hay mensajes recientes (> 0.5 s)
        vel_age = (now - self._last_goal_vel_time).nanoseconds * 1e-9
        if vel_age > 0.5 or self.estado != AUTOMATIC:
            self.goal_vx = 0.0
            self.goal_vy = 0.0
            self.goal_vz = 0.0
            self.goal_vyaw = 0.0

        # Control PID en posición
        comando_roll = self.pid.control_roll(
            self.setpoint_y, self.y_actual, dt)
        comando_pitch = self.pid.control_pitch(
            self.setpoint_x, self.x_actual, dt)
        comando_yaw = self.pid.control_yaw(
            self.setpoint_yaw, self.yaw_actual, dt, self.yaw_rate)

        # Control en Z:
        # En dron real con z_mode == 'native', gaz=0 deja que el Bebop mantenga
        # su altitud estable mediante sonar/barómetro nativos, evitando la deriva
        # de integración que provoca que el dron baje y luego suba.
        if (self.get_parameter('use_real_drone').value
                and self.z_mode == 'native'
                and self.estado == AUTOMATIC):
            # Seguridad: si se desvía más de 0.40m respecto al setpoint, intervenir suavemente
            err_z = self.setpoint_z - self.z_actual
            if abs(err_z) > 0.40:
                comando_z = clamp(err_z * self.pid.kp, -0.20, 0.20)
            else:
                comando_z = 0.0
        else:
            comando_z = self.pid.control_altitud(
                self.setpoint_z, self.z_actual, dt)

        # Transformación mundo → dron (rotar por yaw actual)
        cos_yaw = math.cos(self.yaw_actual)
        sin_yaw = math.sin(self.yaw_actual)
        comando_p = comando_pitch * cos_yaw + comando_roll * sin_yaw
        comando_r = -comando_pitch * sin_yaw + comando_roll * cos_yaw

        # Feedforward de velocidad (marco mundo → marco cuerpo)
        if self.enable_ff and self.estado == AUTOMATIC:
            ff_p = self.goal_vx * cos_yaw + self.goal_vy * sin_yaw
            ff_r = -self.goal_vx * sin_yaw + self.goal_vy * cos_yaw
            comando_p += ff_p * self.ff_gain_xy
            comando_r += ff_r * self.ff_gain_xy
            comando_yaw += self.goal_vyaw * self.ff_gain_yaw

        # Publicar velocidades saturadas
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