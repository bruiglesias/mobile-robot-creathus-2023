#!/usr/bin/env python3

import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def send_goal_and_wait(client, x, y, z, w):
    
    # Create an action client called "move_base" with action definition file "MoveBaseAction"
    
 
    # Waits until the action server has started up and started listening for goals.
    client.wait_for_server()

    # Creates a new goal with the MoveBaseGoal constructor
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    

    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.z = z
    goal.target_pose.pose.orientation.w = w
           

    # Sends the goal to the action server.
    client.send_goal(goal)
    
    # Waits for the server to finish performing the action.
    wait = client.wait_for_result()
    
    

    # If the result doesn't arrive, assume the Server is not available
    
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        # Result of executing the action
        return client.get_result()   

# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    
    try:
        # Initializes a rospy node to let the SimpleActionClient publish and subscribe
        rospy.init_node('movebase_client_py')
        
        x = -8.0
        y = 7.0
        z = 1.0
        w = 0.0

        result = send_goal_and_wait(client, x, y, z, w)

        if result:
            rospy.loginfo("Goal execution done!"+str(result))
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")