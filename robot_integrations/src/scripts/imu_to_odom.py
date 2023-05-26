#!/usr/bin/env python3

import math
from math import sin, cos, pi

import rospy
import tf
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

msg = Imu()

def handle_imu_pose(msg_):
    global msg
    msg = msg_


rospy.init_node('imu_odometry_publisher')

odom_pub = rospy.Publisher("odom_imu", Odometry, queue_size=50)
odom_broadcaster = tf.TransformBroadcaster()

rospy.Subscriber('/imu/data', Imu, handle_imu_pose)

linear_velocity = 0.0
linear_position = 0.0

x = 0.0
y = 0.0
th = 0.0

vx = 0.1
vy = -0.1
vth = 0.1

current_time = rospy.Time.now()
last_time = rospy.Time.now()

r = rospy.Rate(1.0)

while not rospy.is_shutdown():

    current_time = rospy.Time.now()

    # compute odometry in a typical way given the velocities of the robot
    dt = (current_time - last_time).to_sec()
    delta_x = msg.orientation.x
    delta_y = msg.orientation.y
    delta_th = msg.orientation.z

    x += delta_x
    y += delta_y
    th += delta_th

    # since all odometry is 6DOF we'll need a quaternion created from yaw
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

    # next, we'll publish the odometry message over ROS
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "imu_link"

    # set the position
    odom.pose.pose = Pose(Point(0, 0, 0.), Quaternion(*odom_quat))

    # set the velocity
    odom.child_frame_id = "base_link"
    odom.twist.twist = Twist(Vector3(0, 0, 0), Vector3(0, 0, msg.angular_velocity.z))

    # publish the message
    odom_pub.publish(odom)

    last_time = current_time
    r.sleep()



:wq
