#!/usr/bin/env python3
import rospy
import math
from sensor_msgs.msg import LaserScan

def laser_callback(msg):
    # Crie uma cópia da mensagem original
    scan_filtered = msg
    
    limit_upper_angle_degree = 180
    
    init_angle = -135
    final_angle = 135

    # Número de Multiplicações = Ângulo Desejado / Incremento
    
    angle_increment = 0.008726646192371845 # 0.5 deg
    angle_min_desire = math.radians(0)
    angle_max_desire = math.radians(limit_upper_angle_degree)
    
    index_min = round(angle_min_desire / angle_increment)
    index_max = round(angle_max_desire / angle_increment)

    # Atualize o ângulo mínimo, ângulo máximo e ranges para manter apenas os 180 graus desejados
    scan_filtered.angle_max = math.radians(init_angle+limit_upper_angle_degree) # 


    scan_filtered.ranges = msg.ranges[:index_max - 1]


    # Publica os dados filtrados no novo tópico
    pub.publish(scan_filtered)

if __name__ == '__main__':
    rospy.init_node('laser_filter_node')
    # Subscreva o tópico original
    rospy.Subscriber('/front_scan', LaserScan, laser_callback)
    # Crie um publisher para o novo tópico
    pub = rospy.Publisher('/front_scan_180', LaserScan, queue_size=10)

    rospy.spin()
