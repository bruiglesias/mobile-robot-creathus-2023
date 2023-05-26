#!/usr/bin/env python3
import math

import rospy
import tf
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
from pyModbusTCP.client import ModbusClient


def calculate_odometry(ticks_left, ticks_right, wheel_base, wheel_circumference, publish_tf=False):
    # Calcular a distância percorrida por cada roda
    distance_left = ticks_left * (wheel_circumference/1000) # 1000 resolução
    distance_right = ticks_right * (wheel_circumference/1000)
    
    # Calcular o deslocamento linear e angular do robô
    linear_displacement = (distance_left + distance_right) / 2
    angular_displacement = (distance_right - distance_left) / wheel_base
    
    # Atualizar a pose e a velocidade angular do robô
    x += linear_displacement * math.cos(theta)
    y += linear_displacement * math.sin(theta)
    theta += angular_displacement
    
    return x, y, theta

def publish_odometry(x, y, theta, linear_speed, angular_speed, odom_pub):
    odom = Odometry()
    odom.header.stamp = rospy.Time.now()
    odom.header.frame_id = "odom"
    odom.child_frame_id = "base_link"
    
    odom.pose.pose = Pose(Point(x, y, 0), Quaternion(*tf.transformations.quaternion_from_euler(0, 0, theta)))
    odom.twist.twist = Twist(Vector3(linear_speed, 0, 0), Vector3(0, 0, angular_speed))
    
    odom_pub.publish(odom)
    
    # Publincando a tranformação odom para base_link - Implementação específica
    if publish_tf:
        odom_broadcaster = tf.TransformBroadcaster()
        odom_broadcaster.sendTransform(
            (x, y, 0),
            tf.transformations.quaternion_from_euler(0, 0, theta),
            odom.header.stamp,
            "base_link",
            "odom")

if __name__ == '__main__':
    rospy.init_node('encoder_wheels_to_odometry_node')

    # Inicializa variável para escolher publicar TF - Implementação espefífica
    publish_tf = True

    # Inicializar as variáveis de diâmetro da roda e comprimento da base
    wheel_radius = 0.1016 # em metros
    wheel_base = 0.4 # distância entre as rodas em metros
    wheel_circumference = wheel_radius * 2 * math.pi
    
    # Inicializar as variáveis de pose e velocidade
    x = 0
    y = 0
    theta = 0
    linear_speed = 0
    angular_speed = 0
    
    # Inicializar o publisher do tópico "odom"
    odom_pub = rospy.Publisher("odom", Odometry, queue_size=10)
    
    try:
        c = ModbusClient(host="192.168.0.5", port=502, unit_id=1, auto_open=True)

        while not rospy.is_shutdown():

            word1: int = c.read_holding_registers(1)[0]
            word2: int = c.read_holding_registers(0)[0]
            ticks_right = ctypes.c_int32((word1 << 16) | (word2 & 0xFFFF)).value

            word3: int = c.read_holding_registers(5)[0]
            word4: int = c.read_holding_registers(4)[0]
            ticks_left = ctypes.c_int32((word3 << 16) | (word4 & 0xFFFF)).value
            
            # Calcular a odometria com base nos ticks dos encoders
            x, y, theta = calculate_odometry(ticks_left, ticks_right, wheel_base, wheel_circumference, publish_tf)
    
            # Publicar a odometria calculada no tópico "odom"
            publish_odometry(x, y, theta, linear_speed, angular_speed, odom_pub)

    except Exception as e:
        print('Fail to connect PLC')
        print(e)

