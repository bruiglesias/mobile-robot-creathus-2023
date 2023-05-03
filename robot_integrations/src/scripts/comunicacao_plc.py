#!/usr/bin/env python3
import roslib
import rospy
import tf.transformations
from geometry_msgs.msg import Twist
from pyModbusTCP.client import ModbusClient


def callback(msg):

    rospy.loginfo("Received a /cmd_vel message!")
    rospy.loginfo("Linear Components: [%f, %f, %f]"%(msg.linear.x, msg.linear.y, msg.linear.z))
    rospy.loginfo("Angular Components: [%f, %f, %f]"%(msg.angular.x, msg.angular.y, msg.angular.z))

    WHEEL_DIST = 1
    WHEEL_RADIUS = 1.4

    v_l = (msg.linear.x - msg.angular.z * WHEEL_DIST / 2) / WHEEL_RADIUS
    v_r = (msg.linear.x + msg.angular.z * WHEEL_DIST / 2) / WHEEL_RADIUS

    c = ModbusClient(host="192.168.0.5", port=502, unit_id=1, auto_open=True, auto_close=True)

    if c.write_multiple_registers(10, [int(v_l * 10000) , int(v_r * 10000)]):
        print("Convert and Sent to PLC - OK ")
    else:
        print("Error sending to PLC")

def listener():
    rospy.init_node('cmd_vel_listener')
    rospy.Subscriber("/cmd_vel", Twist, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
