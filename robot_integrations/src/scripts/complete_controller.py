#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
import math
from std_msgs.msg import Int32
from sensor_msgs.msg import LaserScan
import time

class SimpleRobotController:
    def __init__(self):
        rospy.init_node('simple_robot_controller', anonymous=True)
        self.vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odom_subscriber = rospy.Subscriber('/odom_imu_encoder', Odometry, self.update_pose)
        self.pose = [None, None, None]
        self.rate = rospy.Rate(10)
        self.trajectory_A = [[-3.2, 0.0, 0.0], [-3.2, -12.0, 90]]  # Trajetória A
        self.trajectory_B = [[-3.2, 0.0, 0.0], [0.0, 0.0, 0.0]]  # Trajetória B
        self.trajectory_C = []  # Trajetória C
        self.trajectory = []

        self.obstacle_distances = [0.65, 0.4, 0.3, 0.3, 0.3, 0.3, 0.2, 0.3, 0.4, 0.4]

        self.kp_dist = 0.5 # Ganho proporcional para distância
        self.kp_ang = 0.3  # Ganho proporcional para ângulo
        self.vel_max = 0.2

        self.destino_atual = None
        self.reached_goal = False
        self.last_point_checked = None

        # Usar no terminal para simular os botões:
        # rostopic pub /buttonA std_msgs/Int32 "data: 0" -r 10
        # rostopic pub /buttonB std_msgs/Int32 "data: 0" -r 10
        # rostopic pub /buttonC std_msgs/Int32 "data: 0" -r 10

        # Assinando os tópicos dos botões
        rospy.Subscriber('/buttonA', Int32, self.botao_callback, callback_args='A')
        rospy.Subscriber('/buttonB', Int32, self.botao_callback, callback_args='B')
        rospy.Subscriber('/buttonC', Int32, self.botao_callback, callback_args='C')

    def botao_callback(self, data, destino):
        if data.data == 1 and self.destino_atual != destino:
            if destino == 'A':
                self.trajectory = self.trajectory_A
            elif destino == 'B':
                self.trajectory = self.trajectory_B
            elif destino == 'C':
                self.trajectory = self.trajectory_C
            self.destino_atual = destino
            print(f"Trajetória alterada para {destino}")

    def update_pose(self, data):
        position = data.pose.pose.position
        orientation = data.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        self.pose[1] = position.y
        self.pose[2] = yaw
        self.pose[0] = position.x

    def calculate_error(self, target_point):

        if self.pose is not None:
            dx = target_point[0] - self.pose[0]
            dy = target_point[1] - self.pose[1]

            error_dist = math.sqrt(dx**2 + dy**2)
            error_ang = math.atan2(dy, dx) - self.pose[2]

            while error_ang > math.pi:
                error_ang -= 2 * math.pi

            while error_ang < -math.pi:
                error_ang += 2 * math.pi

            return error_dist, error_ang
        else:
            return math.inf, math.inf

    def is_at_point(self, point, threshold=0.1):
        error_dist, _ = self.calculate_error(point)
        if error_dist is not None:
            return error_dist < threshold

    def adjust_angle(self, point):
        

            if self.pose is None:
                continue

            # Verifica se há um obstáculo à frente
            if self.check_obstacle_for_areas(): #self.check_obstacle(0.5, 45):
                print("Obstáculo detectado! Parando o robô.")
                self.stop_robot()
                continue

            error_dist, error_ang = self.calculate_error(point)

            if abs(error_ang) < 0.01:
                break

            cmd_vel = Twist()
            cmd_vel.linear.x = 0
            cmd_vel.angular.z = self.kp_ang * error_ang  if self.kp_ang * error_ang <= self.vel_max else self.vel_max
            self.vel_publisher.publish(cmd_vel)
            self.rate.sleep()

    def adjust_final_angle(self, destination_yaw):
        # Ajuste final do ângulo
        while not rospy.is_shutdown():


            if abs(destination_yaw - self.pose[2]) < 0.01:
                break

            destination_yaw_rad = math.radians(destination_yaw) 
            cmd_vel = Twist()
            cmd_vel.linear.x = 0.0 
            cmd_vel.angular.z = self.kp_ang * (destination_yaw - self.pose[2]) if self.kp_ang * (destination_yaw - self.pose[2]) <= self.vel_max else self.vel_max
            self.vel_publisher.publish(cmd_vel)
        
        
    def stop_robot(self):
        cmd_vel = Twist()
        cmd_vel.linear.x = 0
        cmd_vel.angular.z = 0
        self.vel_publisher.publish(cmd_vel)


    def check_obstacle(self, obstacle_distance, angle_range_deg):
        laser_data = rospy.wait_for_message('/scan', LaserScan, timeout=5)  # Aguarda a próxima mensagem do tópico /scan
        if laser_data is not None:
            num_readings = len(laser_data.ranges)
            middle_index = int(num_readings / 2)  # Índice central do scan

            # Calcula os índices do scan correspondentes ao ângulo desejado
            angle_range = int(angle_range_deg / laser_data.angle_increment)
            start_index = max(0, middle_index - angle_range)  # Limita o início ao mínimo de 0
            end_index = min(num_readings - 1, middle_index + angle_range)  # Limita o final ao máximo de num_readings - 1

            # Verifica os obstáculos dentro do intervalo angular especificado
            for i in range(start_index, end_index + 1):
                if laser_data.ranges[i] < obstacle_distance:
                    return True  # Obstáculo detectado a uma distância menor que obstacle_distance
            return False  # Nenhum obstáculo detectado dentro do intervalo angular
        else:
            rospy.logwarn("Não foi possível receber os dados do laser!")


# ... (código anterior)

    def check_obstacle_for_areas(self):

        laser_data = rospy.wait_for_message('/scan', LaserScan, timeout=5)  # Aguarda a próxima mensagem do tópico /scan
        if laser_data is not None:

            regions = [
                min(laser_data.ranges[0:52]),
                min(laser_data.ranges[53:105]),
                min(laser_data.ranges[106:158]),
                min(laser_data.ranges[159:211]),
                min(laser_data.ranges[212:264]),
                min(laser_data.ranges[265:317]),
                min(laser_data.ranges[318:370]),
                min(laser_data.ranges[371:423]),
                min(laser_data.ranges[424:476]),
                min(laser_data.ranges[477:529])
            ]

            for index, region in enumerate(regions):
                limit = self.obstacle_distances[index]
                #print(index)
                #print(region, limit, region <= limit)
                #print()
                if region <= limit:
                    print("O robô parou possivel colisão na região: ", index, " limite de aprox: ", limit, "m obstaculo: ", region, "m")
                    return True

            return False

        else:
            rospy.logwarn("Não foi possível receber os dados do laser!")

# ... (restante do código)

    def follow_trajectory(self):

        rate = rospy.Rate(1)  # Taxa de execução do loop (1 Hz)

        while not rospy.is_shutdown():

            if len(self.trajectory) > 0:

                for index, point in enumerate(self.trajectory):
                    print(point)

                    while not self.is_at_point(point):

                        self.adjust_angle(point)

                        #time.sleep(1)

                        while not rospy.is_shutdown() and not self.is_at_point(point):

                            if self.pose is None:
                                continue

                            # Verifica se há um obstáculo à frente
                            if self.check_obstacle_for_areas(): #self.check_obstacle(0.5, 45):
                                print("Obstáculo detectado! Parando o robô.")
                                self.stop_robot()
                                continue

                            error_dist, error_ang = self.calculate_error(point)

                            cmd_vel = Twist()
                            if True: #index == len(self.trajectory) - 1:
                                cmd_vel.linear.x = self.kp_dist * error_dist if self.kp_dist * error_dist <= self.vel_max else self.vel_max
                                cmd_vel.angular.z = self.kp_ang * error_ang  if self.kp_ang * error_ang <= self.vel_max else self.vel_max
                                self.vel_publisher.publish(cmd_vel)
                                self.rate.sleep()
                            else:
                                cmd_vel.linear.x = self.vel_max
                                cmd_vel.angular.z = 0
                                self.vel_publisher.publish(cmd_vel)
                                self.rate.sleep()

                        if index == len(self.trajectory) - 1:
                            self.stop_robot()
                            self.rate.sleep()

                            self.adjust_final_angle(point[2])
                            self.stop_robot()
                            self.rate.sleep()
                            print("Reached point: ", point)
                            self.trajectory = []


                        self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = SimpleRobotController()
        controller.follow_trajectory()
    except rospy.ROSInterruptException:
        pass
