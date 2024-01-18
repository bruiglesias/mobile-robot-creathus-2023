#!/usr/bin/env python3

import rospy
from pyModbusTCP.client import ModbusClient
import ctypes
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from geometry_msgs.msg import Quaternion, Point, Twist
from math import sin, cos
import math

def calculate_odometry():
    global ticks_left, ticks_right, x, y, theta, prev_x, prev_y, prev_time, prev_theta
    global linear_speed, angular_speed

   
if __name__ == '__main__':

    #rospy.init_node('read_value_clp')



    # Variáveis para armazenar os ticks dos encoders
    global ticks_left, ticks_right
    ticks_left = 0
    ticks_right = 0

    # Parâmetros do robô diferencial (ajuste de acordo com o seu robô)

    global wheel_radius, wheel_base, encoder_resolution
    wheel_radius = 0.1016  # Raio da roda em metros
    wheel_base = 0.48  # Distância entre as rodas em metros
    encoder_resolution = 1000  # Resolução dos encoders incrementais em pulsos por rotação

    # Variáveis para armazenar a odometria do robô
    global x, y, theta
    x = 0.0
    y = 0.0
    theta = 0.0

    # Variáveis para armazenar a posição anterior do robô
    global prev_x, prev_y, prev_theta
    prev_x = 0.0
    prev_y = 0.0
    prev_theta = 0.0

    # Variáveis para o cálculo do delta_t
    global prev_time
    #prev_time = rospy.Time.now()

    global linear_speed, angular_speed
    linear_speed = 0
    angular_speed = 0


    # Ler os ticks dos encoders - Implementação espefífica
    try:
        c = ModbusClient(host="192.168.0.5", port=502, unit_id=1, auto_open=True)
        #odom_pub = rospy.Publisher('encoder_odom', Odometry, queue_size=10)
        
       # while not rospy.is_shutdown():

        word_1: int = c.read_holding_registers(1)[0]
        word_0: int = c.read_holding_registers(0)[0]
        ticks_encoder_1 = ctypes.c_int32((word_1 << 16) | (word_0 & 0xFFFF)).value

        word_5: int = c.read_holding_registers(5)[0]
        word_4: int = c.read_holding_registers(4)[0]
        ticks_encoder_2 = ctypes.c_int32((word_5 << 16) | (word_4 & 0xFFFF)).value
            
            
        ticks_left = ticks_encoder_1
        ticks_right = ticks_encoder_2

        print("word_0 ", word_0)
        print("word_1 ", word_1)
        print("word_4 ", word_4)
        print("word_5 ", word_5)
        
        word_16: int = c.read_holding_registers(16)[0]
        print("word_16 ", word_16)


        word_17: int = c.read_holding_registers(17)[0]
        print("word_17 ", word_17)


        word_18: int = c.read_holding_registers(18)[0]
        print("word_18 ", word_18)

        word_19: int = c.read_holding_registers(19)[0]
        print("word_19 ", word_19)

        word_20: int = c.read_holding_registers(20)[0]
        print("word_20 ", word_20)

            #calculate_odometry()

            #odom = publish_odometry()

            # Publicar a mensagem de odometria no tópico "odom"
            # odom_pub.publish(odom)

            # Ticks
        print('encoder 1: ', ticks_encoder_1, ' encoder_2: ', ticks_encoder_2)
            
            
    except Exception as e:
        print('Fail to connect PLC')
        print(e)
