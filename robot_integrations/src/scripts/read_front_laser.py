#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan

angle = 265  # Defina o ângulo desejado em graus aqui (250 ou 270)

def front_laser_callback(data):
    global angle

    # Recebe os dados do tópico /front_laser
    new_scan = LaserScan()

    # Cálculo para determinar o número de leituras para o ângulo especificado
    angle_range = data.angle_max - data.angle_min  # Calcula a amplitude do ângulo
    num_readings = int(angle_range / data.angle_increment * (angle / 360.0))  # Calcula o número de leituras para o ângulo desejado

    new_scan.header = data.header  # Utiliza o cabeçalho original
    new_scan.angle_min = data.angle_min
    new_scan.angle_max = data.angle_min + (angle * 3.14159 / 180)  # Converte o ângulo para radianos
    new_scan.angle_increment = data.angle_increment
    new_scan.time_increment = data.time_increment
    new_scan.scan_time = data.scan_time
    new_scan.range_min = data.range_min
    new_scan.range_max = data.range_max


    x = int((len(data.ranges) * angle) / 270)
    new_scan.ranges = data.ranges[:x]  # Publica as leituras para o ângulo especificado

    # Publica os dados ajustados no tópico /scan
    scan_pub.publish(new_scan)

def laser_scan_publisher():
    rospy.init_node('laser_scan_node', anonymous=True)
    rospy.Subscriber('/front_scan', LaserScan, front_laser_callback)
    global scan_pub
    scan_pub = rospy.Publisher('/scan', LaserScan, queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    try:
        laser_scan_publisher()
    except rospy.ROSInterruptException:
        pass
