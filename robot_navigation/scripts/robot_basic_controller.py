#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

class SimpleRobotController:
    def _init_(self):
        rospy.init_node('simple_robot_controller')
        self.vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odom_subscriber = rospy.Subscriber('/odom_imu_encoder', Odometry, self.update_pose)
        self.pose = None
        self.rate = rospy.Rate(10)
        self.trajectory = [...]  # Sua trajetória aqui
        self.kp_dist = 1.0  # Ganho proporcional para distância
        self.kp_ang = 1.0  # Ganho proporcional para ângulo

    def update_pose(self, data):
        position = data.pose.pose.position
        orientation = data.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        self.pose = (position.x, position.y, yaw)

    def follow_trajectory(self):
        for point in self.trajectory:
            while not rospy.is_shutdown() and not self.is_at_point(point):
                if self.pose is None:
                    continue
                error_dist, error_ang = self.calculate_error(point)
                cmd_vel = Twist()
                cmd_vel.linear.x = self.kp_dist * error_dist
                cmd_vel.angular.z = self.kp_ang * error_ang
                self.vel_publisher.publish(cmd_vel)
                self.rate.sleep()

    def calculate_error(self, target_point):
        dx = target_point[0] - self.pose[0]
        dy = target_point[1] - self.pose[1]
        error_dist = math.sqrt(dx*2 + dy*2)
        error_ang = math.atan2(dy, dx) - self.pose[2]
        return error_dist, error_ang

    def is_at_point(self, point, threshold=0.1):
        error_dist, _ = self.calculate_error(point)
        return error_dist < threshold

if __name__ == '_main_':
    controller = SimpleRobotController()
    try:
        controller.follow_trajectory()
    except rospy.ROSInterruptException:
        pass
