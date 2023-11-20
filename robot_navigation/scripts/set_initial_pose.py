#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

def publish_initial_pose():

    rospy.init_node('initial_pose_publisher', anonymous=True)
    pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
    # Crie uma mensagem de PoseWithCovarianceStamped com as informações de pose desejadas
    initial_pose = PoseWithCovarianceStamped()
    initial_pose.header.stamp = rospy.Time.now()
    initial_pose.header.frame_id = "map"  # Substitua pelo frame_id apropriado
    initial_pose.pose.pose.position.x = 0.0  # Substitua pelas coordenadas desejadas
    initial_pose.pose.pose.position.y = 0.0
    initial_pose.pose.pose.position.z = 0.0
    initial_pose.pose.pose.orientation.x = 0.0
    initial_pose.pose.pose.orientation.y = 0.0
    initial_pose.pose.pose.orientation.z = 0.0
    initial_pose.pose.pose.orientation.w = 1.0

    # Publica a mensagem uma única vez
    for i in range(10):
        pub.publish(initial_pose)
        rospy.sleep(0.1)  # Aguarda um curto período para garantir que a mensagem seja publicada

if __name__ == '__main__':
    try:
         publish_initial_pose()
    except rospy.ROSInterruptException as e:
        print(e)
