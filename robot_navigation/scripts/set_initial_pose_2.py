#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

def publish_initial_pose():
    rospy.init_node('initial_pose_publisher', anonymous=True)
    pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)

    rate = rospy.Rate(1)  # Taxa de publicação de 1 Hz (1 vez por segundo)

    while not rospy.is_shutdown():
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

        # Publica a mensagem
        pub.publish(initial_pose)

        rate.sleep()

if __name__ == '__main__':
    try:
        publish_initial_pose()
    except rospy.ROSInterruptException:
        pass
