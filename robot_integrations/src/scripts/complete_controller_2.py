#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
import math
from std_msgs.msg import Int32
from sensor_msgs.msg import LaserScan

class SimpleRobotController:
    def _init_(self):
        # Inicializa o nó ROS
        rospy.init_node('simple_robot_controller', anonymous=True)

        # Publicadores e Subscritores
        self.vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odom_subscriber = rospy.Subscriber('/odom_imu_encoder', Odometry, self.update_pose)

        # Variáveis de estado
        self.pose = None
        self.rate = rospy.Rate(10)
        self.trajectory = []  # Trajetória a seguir
        self.obstacle_distances = [0.8, 0.6, 0.3, 0.3, 0.3, 0.3, 0.2, 0.3, 0.4, 0.4]
        self.kp_dist = 0.2  # Ganho proporcional para distância
        self.kp_ang = 0.2  # Ganho proporcional para ângulo
        self.vel_max = 0.1  # Velocidade máxima

        # Subscritores dos botões para iniciar trajetórias
        rospy.Subscriber('/buttonA', Int32, self.button_callback, callback_args='A')
        rospy.Subscriber('/buttonB', Int32, self.button_callback, callback_args='B')
        rospy.Subscriber('/buttonC', Int32, self.button_callback, callback_args='C')

    def button_callback(self, data, destination):
        # Define a trajetória com base no botão pressionado
        if data.data == 1:
            if destination == 'A':
                self.trajectory = [[-1.0, 0.0], [-1.7, 0.0, 1.57]]  # Exemplo de trajetória A
            elif destination == 'B':
                self.trajectory = [[-1.0, 0.0], [0.0, 0.0, 0.0]]  # Exemplo de trajetória B
            elif destination == 'C':
                self.trajectory = []  # Exemplo de trajetória C
            print(f"Trajetória alterada para {destination}")

    def update_pose(self, data):
        # Atualiza a pose do robô com base nos dados da odometria
        position = data.pose.pose.position
        orientation = data.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        self.pose = (position.x, position.y, yaw)

    def calculate_error(self, target_point):
        # Calcula o erro de distância e ângulo até o ponto alvo
        if self.pose is not None:
            dx = target_point[0] - self.pose[0]
            dy = target_point[1] - self.pose[1]
            error_dist = math.sqrt(dx**2 + dy**2)
            error_ang = math.atan2(dy, dx) - self.pose[2]
            error_ang = self.normalize_angle(error_ang)
            return error_dist, error_ang
        else:
            return math.inf, math.inf

    def normalize_angle(self, angle):
        # Normaliza um ângulo para estar entre -pi e pi
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def calculate_error_angle(self, target_point):
        # Calcula apenas o erro angular em relação ao ponto alvo
        if self.pose is not None and len(target_point) == 3:
            target_theta = target_point[2]
            error_ang = target_theta - self.pose[2]
            error_ang = self.normalize_angle(error_ang)
            return error_ang
        else:
            return None

    def follow_trajectory(self):
        # Segue a trajetória definida
        while not rospy.is_shutdown():
            if len(self.trajectory) > 0:
                for index, point in enumerate(self.trajectory):
                    while not self.is_at_point(point):
                        if index == len(self.trajectory) - 1:  # Último ponto
                            error_ang = self.calculate_error_angle(point)
                            if error_ang is not None:
                                self.adjust_angle_to_final_target(error_ang)
                            if self.is_at_point(point, threshold=0.1) and abs(error_ang) < 0.01:
                                self.stop_robot()
                                print("Reached and aligned at final point: ", point)
                                self.trajectory = []
                                return
                        else:
                            self.adjust_movement_to_point(point)

    def is_at_point(self, point, threshold=0.1):
        # Verifica se o robô chegou perto de um ponto
        error_dist, _ = self.calculate_error(point)
        return error_dist < threshold

    def adjust_angle_to_final_target(self, error_ang):
        # Ajusta apenas o ângulo do robô para o último ponto
        cmd_vel = Twist()
        cmd_vel.linear.x = 0
        cmd_vel.angular.z = self.kp_ang * error_ang if self.kp_ang * error_ang <= self.vel_max else self.vel_max
        self.vel_publisher.publish(cmd_vel)

    def adjust_movement_to_point(self, point):
        # Ajusta o movimento do robô para um ponto não final
        error_dist, error_ang = self.calculate_error(point)
        cmd_vel = Twist()
        cmd_vel.linear.x = self.kp_dist * error_dist if self.kp_dist * error_dist <= self.vel_max else self.vel_max
        cmd_vel.angular.z = self.kp_ang * error_ang if self.kp_ang * error_ang <= self.vel_max else self.vel_max
        self.vel_publisher.publish(cmd_vel)

    def stop_robot(self):
        # Para o robô
        cmd_vel = Twist()
        cmd_vel.linear.x = 0
        cmd_vel.angular.z = 0
        self.vel_publisher.publish(cmd_vel)

if __name__ == '_main_':
    try:
        controller = SimpleRobotController()
        controller.follow_trajectory()
    except rospy.ROSInterruptException:
        pass
