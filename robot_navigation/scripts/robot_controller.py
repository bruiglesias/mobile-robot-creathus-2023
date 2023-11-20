#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus

class RobotController:

    def __init__(self) -> None:
        # Create an action client called "move_base" with action definition file "MoveBaseAction"
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    def controller(self):
        
        while True:
            print()
            print("Choice a option:")

            print("1. send_goal_and_wait")
            print("2. send_goal_and_no_wait")
            print("3. get current status")
            print("4. cancel current goal")
            
            choice = int(input("You choice: "))
            print()

            if choice == 1:
                x, y = str(input("Type X Y coords (0.0 0.0): ")).split(" ")
                self.send_goal_and_wait(x, y)

            elif choice == 2:
                x, y = str(input("Type X Y coords (0.0 0.0): ")).split(" ")
                self.send_goal_and_no_wait(x, y)

            elif choice == 3:
                result = self.client.get_state()

                if result == GoalStatus.PENDING:
                    print()
                    print("PENDING: The goal has yet to be processed by the action server ")
                    print()

                if result == GoalStatus.ACTIVE:
                    print("ACTIVE: The goal is currently being processed by the action server")
                
                if result == GoalStatus.PREEMPTED:
                    print()
                    print("PREEMPTED: The goal received a cancel request after it started executing and has since completed its execution (Terminal State)")
                    print()

                if result == GoalStatus.SUCCEEDED:
                    print()
                    print("SUCCEEDED: The goal was achieved successfully by the action server (Terminal State)")
                    print()

                if result == GoalStatus.ABORTED:
                    print()
                    print("ABORTED: The goal was aborted during execution by the action server due to some failure (Terminal State)")
                    print()

                if result == GoalStatus.REJECTED:
                    print()
                    print("REJECTED: The goal was rejected by the action server without being processed, because the goal was unattainable or invalid (Terminal State)")
                    print()

                if result == GoalStatus.PREEMPTING:
                    print()
                    print("PREEMPTING: The goal received a cancel request after it started executing, and has not yet completed execution")
                    print()

                if result == GoalStatus.RECALLING:
                    print()
                    print("RECALLING: The goal received a cancel request before it started executing, but the action server has not yet confirmed that the goal is canceled")
                    print()

                if result == GoalStatus.RECALLED:
                    print()
                    print("RECALLED: The goal received a cancel request before it started executing, and was successfully cancelled (Terminal State)")
                    print()

                if result == GoalStatus.LOST:
                    print()
                    print("LOST: An action client can determine that a goal is LOST. This should not be sent over the wire by an action server")
                    print()

            elif choice == 4:
                self.cancel_current_goal()

            else:
                break

            choice = None



    def send_goal_and_wait(self, x, y, z=1.0, w=0.0):
    
        # Waits until the action server has started up and started listening for goals.
        self.client.wait_for_server()

        # Creates a new goal with the MoveBaseGoal constructor
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        

        goal.target_pose.pose.position.x = float(x)
        goal.target_pose.pose.position.y = float(y)
        goal.target_pose.pose.orientation.z = float(z)
        goal.target_pose.pose.orientation.w = float(w)
            

        # Sends the goal to the action server.
        self.client.send_goal(goal)
        
        # Waits for the server to finish performing the action.
        wait = self.client.wait_for_result()
        
        # If the result doesn't arrive, assume the Server is not available
        
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            # Result of executing the action
            return self.client.get_result()


    def send_goal_and_no_wait(self, x, y, z=1.0, w=0.0):
    
        # Waits until the action server has started up and started listening for goals.
        self.client.wait_for_server()

        # Creates a new goal with the MoveBaseGoal constructor
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        

        goal.target_pose.pose.position.x = float(x)
        goal.target_pose.pose.position.y = float(y)
        goal.target_pose.pose.orientation.z = float(z)
        goal.target_pose.pose.orientation.w = float(w)
            

        # Sends the goal to the action server.
        self.client.send_goal(goal)

    def cancel_current_goal(self):
        self.client.cancel_goal()


if __name__ == "__main__":
    rospy.init_node('robot_controller_client_py')
    controller = RobotController()
    controller.controller()
