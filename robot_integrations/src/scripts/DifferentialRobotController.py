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
        self.rate = rospy.Rate(50)  # 100 Hz (cada 10 ms)

        # Inicializa os valores dos encoders das rodas direita e esquerda
        self.encoder_right = 0
        self.encoder_left = 0
        self.encoder_dt = 0

        self.Vl = 0
        self.Vr = 0

        # Cria os assinantes para os encoders das rodas direita e esquerda
        rospy.Subscriber('/data/encoder/filtered', Vector3Stamped, self.callbackEncoder, queue_size = 1)

        # Distância entre as rodas (base do robô diferencial)
        self.L = 0.35  # 0.35 metros

        # Raio da roda
        self.R = 0.1016  # 0.1016 metros

        # Ganho proporcional para o controle da malha fechada
        self.Kp = 0.1
        self.Ki = 0.01

        # Variáveis de erro acumulado para o controle integral
        self.error_sum_right = 0
        self.error_sum_left = 0

        self.signal_left = 0
        self.signal_right = 0

        self.min_value_error = -1
        self.max_value_error = 1

        #self.min_value_controll = -1
        #self.max_value_controll = 1

        # Aplicar um Multiplicador e converter para inteiro - Implementação específica
        self.multi = 10000

        # Inciar conexão com CLP
        self.c = ModbusClient(host="192.168.0.5", port=502, unit_id=1, auto_open=True)

        self.last_time = rospy.Time.now()

    def clamp_error(self, value):
        return max(min(value, self.max_value_error), self.min_value_error)

    def clamp_controll(self, value, min_value_controll, max_value_controll):
        return max(min(value, max_value_controll), min_value_controll)

    def callbackEncoder(self, msg):
        # Atualiza a leitura do encoder da roda direita
        self.encoder_left = msg.vector.x
        self.encoder_right = msg.vector.y
        # self.encoder_dt = msg.vector.z

        #rospy.loginfo(" [o] encoder_left: %lf encoder_right: %lf", self.encoder_left, self.encoder_right)

    def cmd_vel_callback(self, msg):
        # Obtém os valores de velocidade linear e angular a partir do comando recebido
        Vref = msg.linear.x  # Velocidade linear de referência em m/s
        Wref = msg.angular.z  # Velocidade angular de referência em rad/s

        # Implementa a cinemática inversa para obter as velocidades das rodas
        # Vl = (2 * Vref - Wref * self.L) / (2 * self.R) # (rad/s)
        # Vr = (2 * Vref + Wref * self.L) / (2 * self.R) # (rad/s)

        self.Vl = ((2 * Vref) - (Wref * self.L)) / 2 # (m/s)
        self.Vr = ((2 * Vref) + (Wref * self.L)) / 2 # (m/s)

        # print(f'DEBUG Vl: {self.Vl}  Vr: {self.Vr} ')
        #rospy.loginfo("")
        #rospy.loginfo(" [*] Vl: %lf Vr: %lf", self.Vl, self.Vr)

    def update_controller(self):
        # Controlador de malha fechada
        error_left = float(self.Vl - self.encoder_left)
        error_right = float(self.Vr - self.encoder_right)

        self.error_sum_left += error_left
        self.error_sum_right += error_right

        self.error_sum_left = self.clamp_error(self.error_sum_left)
        self.error_sum_right = self.clamp_error(self.error_sum_right)

        # print(f'DEBUG error_left: {error_left}  error_right {error_right} error_sum_left {self.error_sum_left} error_sum_right {self.error_sum_right}')
        # rospy.loginfo(" [*] error_left: %lf error_right: %lf", error_left, error_right)

        # Implementa o controle feedforward com malha fechada
        Vcontrol_left = self.Vl + self.Kp * error_left + self.error_sum_left * self.Ki
        Vcontrol_right = self.Vr + self.Kp * error_right + self.error_sum_right * self.Ki

        # Vcontrol_left = self.Vl + self.Kp * error_left
        # Vcontrol_right = self.Vr + self.Kp * error_right

        #Vcontrol_left = self.Vl
        #Vcontrol_right = self.Vr

        #min_value_controll = Vl * 0.5
        #max_value_error = Vl * 0.5

        Vcontrol_left = self.clamp_controll(Vcontrol_left, -abs(self.Vl*3.5), abs(self.Vl*3.5))
        Vcontrol_right = self.clamp_controll(Vcontrol_right, -abs(self.Vr*3.5), abs(self.Vr*3.5))

        if self.Vl == 0:
            Vcontrol_left = 0

        if self.Vr == 0:
            Vcontrol_right = 0

        # Define os comandos de velocidade das rodas direita e esquerda
        cmd_vel_controlled = Twist()
        cmd_vel_controlled.linear.x = Vcontrol_left  # Controle de Velocidade linear da roda direita em m/s
        cmd_vel_controlled.linear.y = Vcontrol_right  # Controle de Velocidade linear da roda esquerda em m/s
        cmd_vel_controlled.linear.z = error_left # tempo sem segundos (s)

        cmd_vel_controlled.angular.x = self.Vl  # Referencia Velocidade linear da roda direita em m/s
        cmd_vel_controlled.angular.y = self.Vr  # Referencia Velocidade linear da roda esquerda em m/s
        cmd_vel_controlled.angular.z = error_right # tempo sem segundos (s)

        # Publica os comandos de velocidade
        self.cmd_vel_pub.publish(cmd_vel_controlled)

        left_w_velocity = abs(int(Vcontrol_left * self.multi))
        right_w_velocity = abs(int(Vcontrol_right * self.multi))

        # Sinal de controle de velocidade das rodas direita e esquerda que vão para o robô
        vel_robot_controlled = Twist()
        vel_robot_controlled.linear.x = left_w_velocity  # Velocidade linear da roda direita em m/s
        vel_robot_controlled.linear.y = right_w_velocity  # Velocidade linear da roda esquerda em m/s
        # vel_robot_controlled.linear.z = self.dt # tempo sem segundos (s)

        # Publica os comandos de velocidade
        self.vel_robot_pub.publish(vel_robot_controlled)

        if self.Vl >= 0:
            self.signal_left = 0
        elif Vcontrol_left < 0:
            self.signal_left = 1

        if self.Vr >= 0:
            self.signal_right = 0
        elif Vcontrol_right < 0:
            self.signal_right = 1


        # left_w_velocity = 200
        # right_w_velocity = 200

        # self.signal_left = 0
        # self.signal_right = 0

        # Enviar as velocidades de giro calculadas para o PLC - Implementação específica
        try:
            self.c.write_multiple_registers(10, [right_w_velocity, left_w_velocity, int(self.signal_left), int(self.signal_right)])


            #print(f'Write in PLC: Left: {left_w_velocity}  Right {right_w_velocity} signal_left {self.signal_left} signal_right {self.signal_right}')
        except Exception as e:
            print(f'Fail to connect PLC Right {right_w_velocity} - Left: {left_w_velocity} ')
            print(e.args)


    def run(self):
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            dt = (current_time - self.last_time).to_sec()
            # rospy.loginfo("dt: %lf", dt)

            self.update_controller()

            self.last_time = current_time

            # Aguarda a próxima iteração
            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = DifferentialRobotController()
        controller.run()
    except rospy.ROSInterruptException:
        pass


