#!/usr/bin/env python3

import rospy
from pyModbusTCP.client import ModbusClient
import ctypes
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from geometry_msgs.msg import Quaternion, Point, Twist
from math import sin, cos

rospy.init_node('read_value_clp')

# Variáveis para armazenar os ticks dos encoders
ticks_left = 0
ticks_right = 0

# Parâmetros do robô diferencial (ajuste de acordo com o seu robô)
wheel_radius = 0.1016  # Raio da roda em metros
wheel_base = 0.48  # Distância entre as rodas em metros
encoder_resolution = 1000  # Resolução dos encoders incrementais em pulsos por rotação

# Variáveis para armazenar a odometria do robô
x = 0.0
y = 0.0
theta = 0.0

# Variáveis para armazenar a posição anterior do robô
prev_x = 0.0
prev_y = 0.0

# Variáveis para o cálculo do delta_t
prev_time = rospy.Time.now()

linear_speed = 0
angular_speed = 0



def calculate_odometry():
    global ticks_left, ticks_right, x, y, theta, prev_x, prev_y, prev_time
    global linear_speed, angular_speed

    # Calcular a distância percorrida por cada roda
    distance_left = (2 * math.pi * wheel_radius * ticks_left) / encoder_resolution
    distance_right = (2 * math.pi * wheel_radius * ticks_right) / encoder_resolution

    # Calcular a variação da posição do robô
    delta_s = (distance_left + distance_right) / 2.0
    delta_theta = (distance_right - distance_left) / wheel_base

    # Calcular o tempo decorrido desde a última atualização
    current_time = rospy.Time.now()
    delta_t = (current_time - prev_time).to_sec()
    prev_time = current_time

    # Atualizar a posição e orientação do robô
    delta_x = delta_s * math.cos(theta)
    delta_y = delta_s * math.sin(theta)
    x = prev_x + delta_x
    y = prev_y + delta_y
    theta += delta_theta

    # Atualizar a posição anterior do robô
    prev_x = x
    prev_y = y

    # Ajustar a velocidade linear e angular com base nos dados dos encoders
    linear_speed = delta_s / delta_t
    angular_speed = delta_theta / delta_t

       

def publish_odometry():
    global x, y, theta, linear_speed, angular_speed

    # Criar mensagem de odometria
    odom = Odometry()
    odom.header = Header()
    odom.header.stamp = rospy.Time.now()
    odom.header.frame_id = "odom"
    odom.child_frame_id = "base_link"

    # Preencher a posição do robô na odometria
    odom.pose.pose.position = Point(x, y, 0.0)
    odom.pose.pose.orientation = Quaternion(0.0, 0.0, math.sin(theta / 2.0), math.cos(theta / 2.0))

    # Preencher a velocidade do robô na odometria
    odom.twist.twist = Twist()
    odom.twist.twist.linear.x = linear_speed
    odom.twist.twist.angular.z = angular_speed

    return odom

if __name__ == '__main__':

      
    # Ler os ticks dos encoders - Implementação espefífica
    try:
        c = ModbusClient(host="192.168.0.5", port=502, unit_id=1, auto_open=True)
        odom_pub = rospy.Publisher('encoder_odom', Odometry, queue_size=10)
        
        while not rospy.is_shutdown():

            word_1: int = c.read_holding_registers(1)[0]
            word_0: int = c.read_holding_registers(0)[0]
            ticks_encoder_1 = ctypes.c_int32((word_1 << 16) | (word_0 & 0xFFFF)).value

            word_5: int = c.read_holding_registers(5)[0]
            word_4: int = c.read_holding_registers(4)[0]
            ticks_encoder_2 = ctypes.c_int32((word_5 << 16) | (word_4 & 0xFFFF)).value
            
            
            ticks_left = 0
            ticks_right = 0

            calculate_odometry()

            odom = publish_odometry()

            # Publicar a mensagem de odometria no tópico "odom"
            odom_pub.publish(odom)

            # Ticks
            print('encoder 1: ', ticks_encoder_1, ' encoder_2: ', ticks_encoder_2)
            
            
    except Exception as e:
        print('Fail to connect PLC')
        print(e)
    
