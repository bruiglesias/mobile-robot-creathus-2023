#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan

def front_scan_callback(data):
    # Esta função é chamada sempre que novos dados são recebidos no tópico /front_scan
    # Você pode processar os dados aqui, se desejar

    # Publica os mesmos dados no tópico /scan
    scan_publisher.publish(data)
    print("Publicando")

if __name__ == '__main__':
    rospy.init_node('scan_relay_node')  # Inicializa o nó com um nome

    # Subscreve o tópico /front_scan e define a função de retorno de chamada (callback)
    rospy.Subscriber('/front_scan', LaserScan, front_scan_callback)

    # Cria um publicador para o tópico /scan
    scan_publisher = rospy.Publisher('/scan', LaserScan, queue_size=10)

    rate = rospy.Rate(10)  # Frequência de publicação (10 Hz)

    while not rospy.is_shutdown():
        rate.sleep()
