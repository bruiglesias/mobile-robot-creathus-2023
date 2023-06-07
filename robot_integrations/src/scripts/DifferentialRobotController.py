#!/usr/bin/env python3

import rospy
import math
from std_msgs.msg import Int16
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3Stamped, Vector3
from pyModbusTCP.client import ModbusClient

class DifferentialRobotController:
    def __init__(self):
        # Inicializa o nó do ROS
        rospy.init_node('differential_robot_controller', 
            log_level=rospy.DEBUG, anonymous=True)

        rospy.loginfo(rospy.get_caller_id() + " {+} Init DifferentialRobotController...")

        # Cria um assinante para ler os comandos de velocidade do robô
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback, queue_size=1)

        # Cria um publicador para enviar os comandos de velocidade para o robô
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel_controlled', Twist, queue_size=1)
        self.vel_robot_pub = rospy.Publisher('/vel_robot_controlled', Twist, queue_size=1)

        # Define a taxa de atualização em Hz
        self.rate = rospy.Rate(100)  # 100 Hz (cada 10 ms)

        # Inicializa os valores dos encoders das rodas direita e esquerda
        self.encoder_right = 0
        self.encoder_left = 0
        self.encoder_dt = 0

        # Cria os assinantes para os encoders das rodas direita e esquerda
        Subscriber('/data/enconder', Vector3Stamped, self.callbackEncoder, queue_size = 1)

        # Distância entre as rodas (base do robô diferencial)
        self.L = 0.35  # 0.35 metros

        # Raio da roda
        self.R = 0.1016  # 0.1016 metros

        # Ganho proporcional para o controle da malha fechada
        self.Kp = 0.1

        # Variáveis de erro acumulado para o controle integral
        self.error_sum_right = 0
        self.error_sum_left = 0

        self.signal_left = 0
        self.signal_right = 0

        # Aplicar um Multiplicador e converter para inteiro - Implementação específica
        self.multi = 10000

        # Inciar conexão com CLP
        self.c = ModbusClient(host="192.168.0.5", port=502, unit_id=1, auto_open=True)

        self.last_time = 0

    def callbackEncoder(self, msg):
        # Atualiza a leitura do encoder da roda direita
        self.encoder_left = msg.vector.x
        self.encoder_right = msg.vector.y
        self.encoder_dt = msg.vector.z

    def cmd_vel_callback(self, msg):

        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()

        # Obtém os valores de velocidade linear e angular a partir do comando recebido
        Vref = msg.linear.x  # Velocidade linear de referência em m/s
        Wref = msg.angular.z  # Velocidade angular de referência em rad/s

        # Implementa a cinemática inversa para obter as velocidades das rodas
        # Vl = (2 * Vref - Wref * self.L) / (2 * self.R) # (rad/s)
        # Vr = (2 * Vref + Wref * self.L) / (2 * self.R) # (rad/s)

        Vl = 2 * Vref - Wref * self.L # (m/s)
        Vr = 2 * Vref + Wref * self.L # (m/s)

        # Controlador de malha fechada
        error_left = Vl - self.encoder_left
        error_right = Vr - self.encoder_right
        
        self.error_sum_left += error_left
        self.error_sum_right += error_right

        self.error_sum_left = clamp(self.error_sum_left, -2, 2)
        self.error_sum_right = clamp(self.error_sum_right, -2, 2)

        # Implementa o controle feedforward com malha fechada
        Vcontrol_left = Vl + self.Kp * error_left + self.error_sum_left
        Vcontrol_right = Vr + self.Kp * error_right + self.error_sum_right

        # Define os comandos de velocidade das rodas direita e esquerda
        cmd_vel_controlled = Twist()
        cmd_vel_controlled.linear.x = Vcontrol_right  # Velocidade linear da roda direita em m/s
        cmd_vel_controlled.linear.y = Vcontrol_left  # Velocidade linear da roda esquerda em m/s
        cmd_vel_controlled.linear.z = self.encoder_dt # tempo sem segundos (s)


        # Publica os comandos de velocidade
        self.cmd_vel_pub.publish(cmd_vel_controlled)

        left_w_velocity = abs(int(Vcontrol_left * multi))
        right_w_velocity = abs(int(Vcontrol_right * multi))

        # Sinal de controle de velocidade das rodas direita e esquerda que vão para o robô
        vel_robot_controlled = Twist()
        vel_robot_controlled.linear.x = left_w_velocity  # Velocidade linear da roda direita em m/s
        vel_robot_controlled.linear.y = right_w_velocity  # Velocidade linear da roda esquerda em m/s
        vel_robot_controlled.linear.z = dt # tempo sem segundos (s)

        if Vcontrol_left > 0:
            self.signal_left = 0
        elif left_wheel_velocity < 0:
            self.signal_left = 1

        if Vcontrol_right > 0:
            self.signal_right = 0
        elif right_wheel_velocity < 0:
            self.signal_right = 1

        # Enviar as velocidades de giro calculadas para o PLC - Implementação específica
        try:
            self.c.write_multiple_registers(10, [left_w_velocity, right_w_velocity, 
                int(self.signal_left), int(self.signal_right)])
            # print(f'Write in PLC: Left: {left_w_velocity}  Right {right_w_velocity} signal_left {signal_left} signal_right {signal_right}')
        except Exception as e: 
            print(f'Fail to connect PLC  Left: {left_w_velocity}  Right {right_w_velocity}')
            print(e.args)

        self.last_time = current_time

    def run(self):
        while not rospy.is_shutdown():
            # Aguarda a próxima iteração
            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = DifferentialRobotController()
        controller.run()
    except rospy.ROSInterruptException:
        pass


#########################################################
# class DifferentialRobotController:
#     def __init__(self):
#         # ...

#         # Parâmetros do filtro de média móvel
#         self.filter_window_size = 5  # Tamanho da janela do filtro
#         self.control_history = [0] * self.filter_window_size  # Histórico dos valores de controle

#         # ...

#     def apply_filter(self, value):
#         # Adiciona o novo valor ao histórico de valores de controle
#         self.control_history.append(value)
#         # Remove o valor mais antigo do histórico
#         self.control_history = self.control_history[1:]

#         # Calcula a média dos valores de controle
#         filtered_value = sum(self.control_history) / self.filter_window_size

#         return filtered_value

#     def cmd_vel_callback(self, msg):
#         # ...

#         # Implementa o controle feedforward com malha fechada
#         Vcontrol_right = Vr + self.Kp * error_right + self.error_sum_right
#         Vcontrol_left = Vl + self.Kp * error_left + self.error_sum_left

#         # Aplica o filtro de média móvel aos valores de controle
#         Vcontrol_right_filtered = self.apply_filter(Vcontrol_right)
#         Vcontrol_left_filtered = self.apply_filter(Vcontrol_left)

#         # Define os comandos de velocidade das rodas direita e esquerda
#         cmd_vel_controlled = Twist()
#         cmd_vel_controlled.linear.x = Vcontrol_right_filtered  # Velocidade linear da roda direita em m/s
#         cmd_vel_controlled.linear.y = Vcontrol_left_filtered  # Velocidade linear da roda esquerda em m/s

#         # ...
