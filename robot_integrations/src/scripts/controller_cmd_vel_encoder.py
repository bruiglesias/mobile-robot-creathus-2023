#!/usr/bin/env python3
  
import rospy
import math
from std_msgs.msg import Int16
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3Stamped, Vector3
from pyModbusTCP.client import ModbusClient
  
  
class RobotController:
  
    def __init__(self):

        self.wheel_radius = 0.1016 # meters
        self.wheel_base = 0.35 # meters

        self.signal_left = 0
        self.signal_right = 0

        self.pub_debug_pid = rospy.Publisher('debug/pid', Vector3, queue_size=10)
        
        # rospy.Subscriber("/odom_imu_encoder",Odometry, self.callbackOdom, queue_size = 10)
        rospy.Subscriber("/data/enconder", Vector3Stamped, self.callbackEncoder, queue_size = 10)

        rospy.Subscriber("/cmd_vel", Twist, self.callbackCmdVel, queue_size = 10)
  
    def callbackCmdVel(self, Twist):
        
        rospy.loginfo(rospy.get_caller_id() + "The velocities are %s", Twist)
        print('Callback executed!')

    def callbackEncoder(self, Vector3Stamped):
        
        rospy.loginfo(rospy.get_caller_id() + "The velocities are %s", Vector3Stamped)
        print('Callback executed!')
  
  
def main(args):
    # initializing node
    rospy.init_node('robot_controller', log_level=rospy.DEBUG, anonymous=True)

    print('Call main function...')
    controller = RobotController()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("ShutDown")

if __name__=='__main__':
    try:
        main(sys.argv)
    except rospy.ROSInterruptException:
        pass