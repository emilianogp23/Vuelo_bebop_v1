import os
import sys 
import csv

# Asegurar acceso a paquetes de rob_env si se ejecuta desde /usr/bin/ros2
venv_site = os.path.expanduser('~/rob_env/lib/python3.12/site-packages')
if os.path.exists(venv_site) and venv_site not in sys.path:
    sys.path.insert(0, venv_site)

from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore
import math


def euler_yaw_from_quaternion(x, y, z, w):
    """Extrae yaw de un cuaternión."""
    t3 = 2.0 * (w * z + x * y)
    t4 = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(t3, t4)



def main ():
    bag_path=os.path.abspath(sys.argv[1])
    csv_file = f"{bag_path}_datos.csv"

    topics = [
        '/bebop1/odom', '/bebop/odom', '/bebop1/pose',
        '/goal', '/flight_state', '/bebop1/cmd_vel', '/bebop1/cmd_vel_raw',
        '/bebop/cmd_vel', '/joy_cmd'
    ]
    print(f'leyendo la base de datos: {bag_path}')
    typestore = get_typestore(Stores.ROS2_HUMBLE)

    with Reader(bag_path) as reader, open(csv_file, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['Tiempo_s', 'Topico', 'X', 'Y', 'Z', 'Yaw', 'Vx', 'Vy', 'Vz', 'V_yaw', 'Estado_vuelo'])

        t0 = None
        for connection, timestamp, rawdata in reader.messages():
            if connection.topic not in topics:
                continue 

            msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
            t = timestamp / 1e9
            if t0 is None:
                t0 = t

            t = round(t - t0, 4)

            if connection.topic in ('/bebop1/odom', '/bebop/odom'):
                x = round(msg.pose.pose.position.x, 4)
                y = round(msg.pose.pose.position.y, 4)
                z = round(msg.pose.pose.position.z, 4)
                qx = msg.pose.pose.orientation.x
                qy = msg.pose.pose.orientation.y
                qz = msg.pose.pose.orientation.z
                qw = msg.pose.pose.orientation.w
                yaw = round(euler_yaw_from_quaternion(qx, qy, qz, qw), 4)

                vx = round(msg.twist.twist.linear.x, 4)
                vy = round(msg.twist.twist.linear.y, 4)
                vz = round(msg.twist.twist.linear.z, 4)
                v_yaw = round(msg.twist.twist.angular.z, 4)

                writer.writerow([t, connection.topic, x, y, z, yaw, vx, vy, vz, v_yaw, ''])

            elif connection.topic == '/bebop1/pose':
                x = round(msg.position.x, 4)
                y = round(msg.position.y, 4)
                z = round(msg.position.z, 4)
                qx = msg.orientation.x
                qy = msg.orientation.y
                qz = msg.orientation.z
                qw = msg.orientation.w
                yaw = round(euler_yaw_from_quaternion(qx, qy, qz, qw), 4)
                writer.writerow([t, 'local_pose', x, y, z, yaw, '', '', '', '', ''])

            elif connection.topic == '/goal':
                x = round(msg.position.x, 4)
                y = round(msg.position.y, 4)
                z = round(msg.position.z, 4)
                writer.writerow([t, 'goal', x, y, z, '', '', '', '', '', ''])

            elif connection.topic in ('/bebop1/cmd_vel', '/bebop1/cmd_vel_raw', '/bebop/cmd_vel', '/joy_cmd'):
                cvx = round(msg.linear.x, 4)
                cvy = round(msg.linear.y, 4)
                cvz = round(msg.linear.z, 4)
                cv_yaw = round(msg.angular.z, 4)
                writer.writerow([t, connection.topic, '', '', '', '', cvx, cvy, cvz, cv_yaw, ''])

            elif connection.topic == '/flight_state':
                writer.writerow([t, 'state', '', '', '', '', '', '', '', '', msg.data])

    print ("Archivo guardado")

if __name__ == '__main__':
    main()
