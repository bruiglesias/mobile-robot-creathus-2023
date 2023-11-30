#!/usr/bin/env python3

import rospy
import json
from nav_msgs.msg import Odometry

class OdometrySaver(object):
    def __init__(self):
        rospy.init_node('odometry_saver')
        self.odom_sub = rospy.Subscriber('/odom_imu_encoder', Odometry, self.odom_callback)
        self.last_odom = None

    def odom_callback(self, odom_msg):
        # Salvar a última posição da odometria
        self.last_odom = odom_msg

    def save_last_odometry_to_json(self, filename):
        if self.last_odom:
            position = self.last_odom.pose.pose.position
            orientation = self.last_odom.pose.pose.orientation

            # Converter quaternião em ângulos de Euler
            import tf
            orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
            roll, pitch, yaw = tf.transformations.euler_from_quaternion(orientation_list)

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
            with open(filename, 'w') as json_file:
                json.dump(data, json_file, indent=4)

if __name__ == '__main__':
    try:
        odometry_saver = OdometrySaver()
        rate = rospy.Rate(10)  # Taxa de salvamento, 10 Hz
        rospy.loginfo("Pose saver started")
        while not rospy.is_shutdown():
            odometry_saver.save_last_odometry_to_json('last_position.json')
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
