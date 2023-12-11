#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Quaternion
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import json

class SaveRobotPosition:
    def __init__(self):
        rospy.init_node('save_robot_position_node', anonymous=True)

        self.rate = rospy.Rate(1)

        # Tópico para receber a posição do robô
        self.robot_position_sub = rospy.Subscriber('/odom_imu_encoder', Odometry, self.robot_position_callback)

        # Tópico para publicar a posição inicial ao AMCL
        self.initial_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
        self.last_robot_position = None

        # Verifica se existe um arquivo com uma última posição salva
        self.check_initial_position()


    def robot_position_callback(self, msg):
        # Atualiza a última posição do robô recebida
        self.last_robot_position = msg.pose.pose

    def check_initial_position(self):
        file_name = '/home/creathus/amr_creathus_ws/src/mobile-robot-creathus-2023/robot_navigation/goals/points_db/last_robot_position.json'

        try:
            with open(file_name, 'r') as file:
                data = json.load(file)
                if data:
                    rospy.loginfo(f'Última posição do arquivo: {data}')

                    # Configura a última posição do arquivo como a posição inicial
                    position_data = data["position"]
                    orientation_data = data["orientation"]

                    x = position_data["x"]
                    y = position_data["y"]
                    z = position_data["z"]

                    roll = orientation_data["roll"]
                    pitch = orientation_data["pitch"]
                    yaw = orientation_data["yaw"]

                    # Configura a posição inicial
                    self.set_initial_pose(x, y, z, roll, pitch, yaw)

        except FileNotFoundError:
            rospy.loginfo(f'Arquivo {file_name} não encontrado. Nenhuma posição inicial definida ainda.')
            self.set_initial_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        except Exception as e:
            rospy.logerr(f'Erro ao verificar o arquivo: {e}')


    def set_initial_pose(self, x, y, z, roll, pitch, yaw):

        # Configura a posição inicial usando /initialpose
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.frame_id = "map"  # Substitua pelo frame_id apropriado

        initial_pose.pose.pose.position.x = x
        initial_pose.pose.pose.position.y = y
        initial_pose.pose.pose.position.z = z


        quaternion = Quaternion(*quaternion_from_euler(roll, pitch, yaw))
        initial_pose.pose.pose.orientation = quaternion

        # Publica a posição inicial para o AMCL
        for i in range(3):
            initial_pose.header.stamp = rospy.Time.now()
            self.initial_pose_pub.publish(initial_pose)
            self.rate.sleep()

        rospy.loginfo('Posição inicial configurada usando o arquivo.')

    def save_position_to_file(self):
        # Verifica se há uma última posição do robô válida
        if self.last_robot_position is not None:
            try:
                # Nome do arquivo JSON para salvar a posição
                file_name = '/home/creathus/amr_creathus_ws/src/mobile-robot-creathus-2023/robot_navigation/goals/points_db/last_robot_position.json'

                # Extrai os valores da posição e orientação do robô
                position = self.last_robot_position.position
                orientation = self.last_robot_position.orientation

                roll, pitch, yaw = euler_from_quaternion([
                    orientation.x,
                    orientation.y,
                    orientation.z,
                    orientation.w
                ])

                # Cria um dicionário com os dados da posição e orientação
                data = {
                    "position": {
                        "x": position.x,
                        "y": position.y,
                        "z": position.z
                    },
                    "orientation": {
                        "roll": roll,
                        "pitch": pitch,
                        "yaw": yaw
                    }
                }

                # Escreve o dicionário no arquivo JSON (sobrescrevendo a posição anterior)
                with open(file_name, 'w') as file:
                    json.dump(data, file)

                # rospy.loginfo(f'Última posição do robô salva no arquivo: {file_name}')

            except IOError as e:
                rospy.logerr(f'Erro ao salvar a posição do robô: {e}')

        else:
            rospy.logwarn('Nenhuma posição do robô recebida ainda.')

    def run(self):
        # Executa indefinidamente até que o nó seja interrompido
        rate = rospy.Rate(1)  # Taxa de execução do loop (1 Hz)

        while not rospy.is_shutdown():
            # Chama a função para salvar a posição do robô no arquivo
            self.save_position_to_file()
            rate.sleep()

if __name__ == '__main__':
    try:
        # Inicializa o nó
        save_position_node = SaveRobotPosition()
        # Executa o nó
        save_position_node.run()
    except rospy.ROSInterruptException:
        pass
