#!/usr/bin/env python3

import math

import Jetson.GPIO as GPIO
import rospy
from geometry_msgs.msg import Point, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry

# declarar as variáveis
diametro_roda = 0.05  # em cm
pulsos_por_volta = 2500
distancia_entre_rodas = 0.15  # em cm
pulsos_esquerda = 0
pulsos_direita = 0


# Configuração dos pinos GPIO
GPIO.setmode(GPIO.BCM)

# Definição dos pinos A, B e Z dos encoders
encoder_esq = {"A": 494, "B": 493}
encoder_dir = {"A": 495, "B": 492}

# Configuração dos pinos como entrada
for pino in encoder_esq.values():
    GPIO.setup(pino, GPIO.IN, pull_up_down=GPIO.PUD_UP)

for pino in encoder_dir.values():
    GPIO.setup(pino, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Variáveis para contagem de pulsos
pulsos_esq = 0
pulsos_dir = 0

# Função para contar os pulsos do encoder esquerdo


def contar_pulsos_esq(channel):
    global pulsos_esq
    if GPIO.input(encoder_esq["B"]) == 1:
        pulsos_esq += 1
    else:
        pulsos_esq -= 1

# Função para contar os pulsos do encoder direito


def contar_pulsos_dir(channel):
    global pulsos_dir
    if GPIO.input(encoder_dir["B"]) == 1:
        pulsos_dir += 1
    else:
        pulsos_dir -= 1

# calcular a distância percorrida por cada roda


def calcular_distancias():
    global pulsos_esq, encoder_dir

    dist_esquerda = (pulsos_esq * math.pi * diametro_roda) / (pulsos_por_volta)
    dist_direita = (encoder_dir * math.pi * diametro_roda) / (pulsos_por_volta)

    return dist_esquerda, dist_direita

# converter a orientação do robô de radianos para graus


def rad2deg(angulo_radianos):
    return angulo_radianos * 180.0 / math.pi


def calcular_odometria(dist_esquerda, dist_direita):
    dist_total = (dist_esquerda + dist_direita) / 2.0
    delta_orientacao = (dist_direita - dist_esquerda) / distancia_entre_rodas
    return dist_total, delta_orientacao


# criar uma mensagem de odometria do ROS
def criar_mensagem_odometria(dist_total, delta_orientacao):
    posicao_x = dist_total * math.cos(delta_orientacao)
    posicao_y = dist_total * math.sin(delta_orientacao)

    msg = Odometry()
    msg.header.stamp = rospy.Time.now()
    msg.pose.pose.position = Point(posicao_x, posicao_y, 0.0)
    quat = Quaternion()
    quat.z = math.sin(delta_orientacao / 2.0)
    quat.w = math.cos(delta_orientacao / 2.0)
    msg.pose.pose.orientation = quat
    msg.twist.twist.linear = Vector3(dist_total, 0.0, 0.0)
    msg.twist.twist.angular.z = rad2deg(delta_orientacao)
    return msg


# Configuração das interrupções nos pinos A dos encoders
GPIO.add_event_detect(encoder_esq["A"], GPIO.BOTH, callback=contar_pulsos_esq)
GPIO.add_event_detect(encoder_dir["A"], GPIO.BOTH, callback=contar_pulsos_dir)


def main():
    rospy.init_node('odom_node', anonymous=True)

    rate = rospy.Rate(10)  # 10 Hz
    publisher_odometria = rospy.Publisher(
        'odom/robot', Odometry, queue_size=10)

    while not rospy.is_shutdown():
        dist_esquerda, dist_direita = calcular_distancias()
        dist_total, delta_orientacao = calcular_odometria(
            dist_esquerda, dist_direita)

        # converter a orientação do robô para graus
        orientacao_graus = rad2deg(delta_orientacao)

        # publicar mensagem de odometria
        mensagem_odometria = criar_mensagem_odometria(
            dist_total, delta_orientacao)

        publisher_odometria.publish(mensagem_odometria)

        rate.sleep()

    GPIO.cleanup()


if __name__ == "__main__":
    main()
