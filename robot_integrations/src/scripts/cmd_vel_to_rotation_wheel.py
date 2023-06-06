#!/usr/bin/env python3
import math

import rospy
from geometry_msgs.msg import Twist
from pyModbusTCP.client import ModbusClient

# Inicializar as variáveis de diâmetro da roda e comprimento da base
global wheel_radius, wheel_base

wheel_radius = 0.1016 # em metros
wheel_base = 0.35 # em metros

signal_left = 0
signal_right = 0

def callback(data):

    global signal_left
    global signal_right

    linear_velocity = data.linear.x
    angular_velocity = data.angular.z
    
    

    # Calcular a velocidade linear para cada roda
    left_wheel_velocity = ( (2 * linear_velocity) - (angular_velocity * wheel_base) )/2
    right_wheel_velocity = ( (2 * linear_velocity) + (angular_velocity * wheel_base) )/2; 

    # Converter a velocidade de giro para RPM (rotações por minuto)
    # left_wheel_velocity_rpm = left_wheel_velocity / (wheel_radius * 2 * math.pi) * 60
    # right_wheel_velocity_rpm = right_wheel_velocity / (wheel_radius * 2 * math.pi) * 60


    # Aplicar um Multiplicador e converter para inteiro - Implementação específica
    multi = 10000



    if left_wheel_velocity > 0:
        signal_left = 0
    elif left_wheel_velocity < 0:
        signal_left = 1


    if right_wheel_velocity > 0:
        signal_right = 0
    elif right_wheel_velocity < 0:
        signal_right = 1

   
    left_w_velocity = abs(int(left_wheel_velocity * multi))
    right_w_velocity = abs(int(right_wheel_velocity * multi))


    # Enviar as velocidades de giro calculadas para o PLC - Implementação específica
    try:
        c = ModbusClient(host="192.168.0.5", port=502, unit_id=1, auto_open=True)
        c.write_multiple_registers(10, [left_w_velocity, right_w_velocity, int(signal_left), int(signal_right)])
        print(f'Write in PLC: Left: {left_w_velocity}  Right {right_w_velocity} signal_left {signal_left} signal_right {signal_right}')
    except Exception as e: 
        print(f'Fail to connect PLC  Left: {left_w_velocity}  Right {right_w_velocity}')
        print(e.args)


if __name__ == '__main__':
    rospy.init_node('cmd_vel_to_wheel_velocity_node')
     
    # Inscrever-se ao tópico "cmd_vel"
    rospy.Subscriber("cmd_vel", Twist, callback, queue_size=10)
    
    rospy.spin()