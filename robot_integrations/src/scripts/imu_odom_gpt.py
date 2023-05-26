#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose, Point, Twist, Vector3, Quaternion
import tf
import math

# Variáveis para armazenar a posição e orientação da odometria
x = 0.0
y = 0.0
theta = 0.0

# Variáveis para armazenar os dados anteriores da IMU
prev_acceleration_x = 0.0
prev_acceleration_y = 0.0
prev_time = None

def imu_callback(data):
    # Aqui você processa os dados recebidos da IMU e calcula a odometria
    # Supondo que você tenha a posição x, y e orientação theta da odometria

    global x, y, theta, prev_acceleration_x, prev_acceleration_y, prev_time
    
    # Cria um publisher para o tópico "/odom"
    odom_pub = rospy.Publisher("/odom_imu", Odometry, queue_size=10)

    # Obtém a aceleração linear da IMU
    acceleration_x = data.linear_acceleration.x
    acceleration_y = data.linear_acceleration.y
    odom_quaternion = tf.transformations.euler_from_quaternion([data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w])
    
    roll_deg = math.degrees(odom_quaternion[0])
    pitch_deg = math.degrees(odom_quaternion[1])
    yaw_deg = math.degrees(odom_quaternion[2])

    # Obtém o tempo atual
    current_time = rospy.Time.now()
    
    if prev_time is not None:
        # Calcula o intervalo de tempo entre as leituras
        dt = (current_time - prev_time).to_sec()
        
        # Calcula a velocidade usando a média das acelerações
        velocity_x = (acceleration_x + prev_acceleration_x) / 2.0 * dt
        velocity_y = (acceleration_y + prev_acceleration_y) / 2.0 * dt
        
        # Atualiza a posição usando a velocidade média
        x += velocity_x * dt
        y += velocity_y * dt
    
    # Atualiza os dados anteriores da IMU e o tempo
    prev_acceleration_x = acceleration_x
    prev_acceleration_y = acceleration_y
    prev_time = current_time

    # Cria uma mensagem do tipo Odometry
    odom = Odometry()
    
    # Preenche os campos da odometria
    odom.header.stamp = rospy.Time.now()
    odom.header.frame_id = "imu_link"
    odom.child_frame_id = "base_link"
    odom.pose.pose = Pose(Point(0, 0, 0.), Quaternion(data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w))
    
    odom.twist.twist.angular.z = yaw_deg
    # Publica a odometria no tópico "/odom"
    odom_pub.publish(odom)

def odom_publisher():
    rospy.init_node('imu_odom_publisher', anonymous=True)
    
    # Cria um subscriber para o tópico "/imu/data" e define a função de callback como imu_callback
    rospy.Subscriber("/imu/data", Imu, imu_callback)
    
    # Cria um publisher para o tópico "/odom"
    #odom_pub = rospy.Publisher("/odom_imu", Odometry, queue_size=10)

    rospy.spin()

if __name__ == '__main__':
    try:
        odom_publisher()
    except rospy.ROSInterruptException:
        pass

