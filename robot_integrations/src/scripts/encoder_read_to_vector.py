#!/usr/bin/env python3

import math
from math import sin, cos, pi
from pyModbusTCP.client import ModbusClient
import ctypes
import rospy
import tf
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3Stamped, Vector3

# Parameters
R = 0.1016
TPR_L = 1000
TPR_R = 1916
PI = 3.14159265358979323846

left_ticks = 0
right_ticks = 0
last_left_ticks = 0
last_right_ticks = 0

# Parâmetros do filtro de média móvel
filter_window_size = 200  # Tamanho da janela do filtro
control_history = [0] * filter_window_size  # Histórico dos valores de controle

def apply_filter(value):

    global control_history

    # Adiciona o novo valor ao histórico de valores de controle
    control_history.append(value)

    # Remove o valor mais antigo do histórico
    control_history = control_history[1:]

    # Calcula a média dos valores de controle
    filtered_value = sum(control_history) / filter_window_size

    return filtered_value

def encoder_reader(c):
    
    global left_ticks, right_ticks
    
    word_1: int = c.read_holding_registers(1)[0]
    word_0: int = c.read_holding_registers(0)[0]
    ticks_encoder_1 = ctypes.c_int32((word_1 << 16) | (word_0 & 0xFFFF)).value

    word_5: int = c.read_holding_registers(5)[0]
    word_4: int = c.read_holding_registers(4)[0]
    ticks_encoder_2 = ctypes.c_int32((word_5 << 16) | (word_4 & 0xFFFF)).value

    # print(ticks_encoder_1, ticks_encoder_2)
    left_ticks = ticks_encoder_2
    right_ticks = ticks_encoder_1


rospy.init_node('encoder_publisher')

encoder_pub = rospy.Publisher("/data/encoder", Vector3Stamped, queue_size=1)
tick_pub = rospy.Publisher("/data/tick_encoder", Vector3, queue_size=1)

encoder_filtered_pub = rospy.Publisher("/data/encoder/filtered", Twist, queue_size=1)

current_time = rospy.Time.now()
last_time = rospy.Time.now()

r = rospy.Rate(100)  # 100 Hz (cada 10 ms)


try: 

    c = ModbusClient(host="192.168.0.5", port=502, unit_id=1, auto_open=True)

    while not rospy.is_shutdown():

        current_time = rospy.Time.now()

        encoder_reader(c)

        delta_L = left_ticks - last_left_ticks
        delta_R = right_ticks - last_right_ticks

        resolution_left = (2 * PI) / TPR_L # rad
        resolution_right = (2 * PI) / TPR_R # rad

        dt = (current_time - last_time).to_sec()

        dl = (resolution_left * delta_L * R) / dt # m/s
        dr = (resolution_right * delta_R * R) / dt # m/s

        encoder = Vector3Stamped()
        encoder.header.frame_id = "data_encoder"
        encoder.vector.x = dl
        encoder.vector.y = dr
        encoder.vector.z = dt

        encoder_pub.publish(encoder)

        # Aplica o filtro de média móvel aos valores de encoder
        vel_encoder_left_filtered = apply_filter(dl)
        vel_enconder_right_filtered = apply_filter(dr)

        # Publica os valores velocidade das rodas direita e esquerda filtrados
        encoder_filtered = Twist()
        encoder_filtered.linear.x = vel_encoder_left_filtered  # Velocidade linear da roda direita em m/s
        encoder_filtered.linear.y = vel_enconder_right_filtered  # Velocidade linear da roda esquerda em m/s

        encoder_filtered_pub.publish(encoder_filtered)

        # Pulblicar os ticks das rodas para debug
        tick = Vector3()
        tick.x = delta_L
        tick.y = delta_R
        tick.z = dt

        tick_pub.publish(tick)

        last_left_ticks = left_ticks
        last_right_ticks = right_ticks
        last_time = current_time
        
        r.sleep()

except Exception as e:
    print('Fail to connect PLC')
    print(e)
