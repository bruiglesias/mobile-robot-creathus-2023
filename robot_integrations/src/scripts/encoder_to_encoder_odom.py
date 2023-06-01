#!/usr/bin/env python3

import math
from math import sin, cos, pi
from pyModbusTCP.client import ModbusClient
import ctypes
import rospy
import tf
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

# Parameters
wheel_track = 0.24
wheel_radius = 0.1016
TPR_L = 1000
TPR_R = 1916

left_ticks = 0
right_ticks = 0
last_left_ticks = 0
last_right_ticks = 0

x = 0.0
y = 0.0
th = 0.0

vx = 0.0
vy = 0.0
vth = 0.0

def encoder_reader(c):
    
    global left_ticks, right_ticks
    
    word_1: int = c.read_holding_registers(1)[0]
    word_0: int = c.read_holding_registers(0)[0]
    ticks_encoder_1 = ctypes.c_int32((word_1 << 16) | (word_0 & 0xFFFF)).value

    word_5: int = c.read_holding_registers(5)[0]
    word_4: int = c.read_holding_registers(4)[0]
    ticks_encoder_2 = ctypes.c_int32((word_5 << 16) | (word_4 & 0xFFFF)).value
    # print(ticks_encoder_1, ticks_encoder_2)
    left_ticks = ticks_encoder_2
    right_ticks = ticks_encoder_1


rospy.init_node('odometry_publisher')

odom_pub = rospy.Publisher("odom_enconder", Odometry, queue_size=50)
odom_broadcaster = tf.TransformBroadcaster()

current_time = rospy.Time.now()
last_time = rospy.Time.now()

r = rospy.Rate(10)


try: 

    c = ModbusClient(host="192.168.0.5", port=502, unit_id=1, auto_open=True)

    while not rospy.is_shutdown():

        current_time = rospy.Time.now()

        encoder_reader(c)

        delta_L = left_ticks - last_left_ticks
        delta_R = right_ticks - last_right_ticks
        dl = 2 * pi * wheel_radius * delta_L / TPR_L
        dr = 2 * pi * wheel_radius * delta_R / TPR_R
        dc = (dl + dr) / 2
        dt = (current_time - last_time).to_sec()
        dth = ( dr - dl) / wheel_track

        if dr == dl:
            dx = dr * cos(th)
            dy = dr * sin(th)

        else:
            radius = dc / dth

            iccX = x - radius * sin(th)
            iccY = y + radius * cos(th)

            dx = cos(dth) * (x-iccX) - sin(dth) * (y-iccY) + iccX - x
            dy = sin(dth) * (x-iccX) + cos(dt) * (y-iccY) + iccY - y

        x += dx  
        y += dy 
        th = (th + dth) % (2 * pi)

        odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

        # first, we'll publish the transform over tf
        odom_broadcaster.sendTransform((x, y, 0.), odom_quat, current_time, "base_link", "odom")

        # next, we'll publish the odometry message over ROS
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"

        odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

        if dt > 0:
            vx = dx / dt
            vy = dy / dt
            vth = dth / dt

        odom.child_frame_id = "base_link"
        odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

        odom_pub.publish(odom)

        last_left_ticks = left_ticks
        last_right_ticks = right_ticks
        last_time = current_time
        
        r.sleep()

except Exception as e:
    print('Fail to connect PLC')
    print(e)
