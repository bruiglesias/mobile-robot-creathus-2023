#!/usr/bin/env python

import rospy
import json
from geometry_msgs.msg import PoseWithCovarianceStamped

class InitialPoseSetter(object):
    def __init__(self):
        rospy.init_node('initial_pose_setter')
        self.initial_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)

    def set_initial_pose_from_json(self, filename):
        try:
            with open(filename, 'r') as json_file:
                data = json.load(json_file)
                initial_pose = PoseWithCovarianceStamped()
                initial_pose.header.stamp = rospy.Time.now()
                initial_pose.header.frame_id = 'map'
                initial_pose.pose.pose.position.x = data["position"]["x"]
                initial_pose.pose.pose.position.y = data["position"]["y"]
                initial_pose.pose.pose.position.z = data["position"]["z"]
                # Configure a orientação a partir dos ângulos de Euler (roll, pitch, yaw)
                import tf
                orientation_quaternion = tf.transformations.quaternion_from_euler(
                    data["orientation"]["roll"],
                    data["orientation"]["pitch"],
                    data["orientation"]["yaw"]
                )
                initial_pose.pose.pose.orientation.x = orientation_quaternion[0]
                initial_pose.pose.pose.orientation.y = orientation_quaternion[1]
                initial_pose.pose.pose.orientation.z = orientation_quaternion[2]
                initial_pose.pose.pose.orientation.w = orientation_quaternion[3]
                for i in range(10):
                    self.initial_pose_pub.publish(initial_pose)
                rospy.loginfo('Configurando a posição e orientação inicial a partir de ' + filename)
        except FileNotFoundError:
            rospy.logerr('Arquivo ' + filename + ' não encontrado.')

if __name__ == '__main__':
    try:
        initial_pose_setter = InitialPoseSetter()
        initial_pose_setter.set_initial_pose_from_json('last_position.json')
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
