#!/usr/bin/env python3

import rospy
import json
from std_srvs.srv import Empty, EmptyResponse
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from robot_navigation.srv import Point, PointRequest, PointResponse

robot_pose = Pose()


def read_json():
    try:
        with open("/home/creathus/amr_creathus_ws/src/mobile-robot-creathus-2023/robot_navigation/goals/points_db/points.json", "r") as file:
            data = json.load(file)
            return data
    except:
        return []

def save_json(points_dict):
    with open("/home/creathus/amr_creathus_ws/src/mobile-robot-creathus-2023/robot_navigation/goals/points_db/points.json", "w") as outfile:
        json.dump(points_dict, outfile)

def print_point(point):
    print("Point Name: ", point["point_name"])
    print("Coords: ")
    print("     position: ", point['coords']['position'])
    print("     orientation: ", point['coords']['orientation'])
    print()

def pose_to_dict(pose: Pose):

    pose_dict = {
        "position": {
            "x": pose.position.x,
            "y": pose.position.y,
            "z": pose.position.x
        },
        "orientation":{
            "x": pose.orientation.x,
            "y": pose.orientation.y,
            "z": pose.orientation.z,
            "w": pose.orientation.w
        }
    }

    return pose_dict

def current_position_callback(request):
    print()
    print("Current Robot Pose:")
    print(robot_pose)
    print()

    return EmptyResponse()

def all_points_callback(request):
    print()
    print("Points Stored:")
    print()
    points = read_json()
    for point in points:
        print_point(point)

    return EmptyResponse()


def add_new_point_callback(request: PointRequest):
    print()
    print("Saving Point:")

    points = read_json()
    point = pose_to_dict(robot_pose)

    if not points:
        points = []
        points.append({
            "point_name":request.name,
            "coords" :point
            })

        save_json(points)
    else:

        for p in points:
            if p['point_name'] == request.name:
                print("Error: point name already exists.")
                return PointResponse(False)

        points.append({
            "point_name":request.name,
            "coords":point
            })

        save_json(points)

    print(robot_pose)
    print()
    return PointResponse(True)

def delete_point_callback(request: PointRequest):
    print()
    print("Deleting Point:")

    points = read_json()

    if not points:
        points = []
        print("Error: The point not exists.")
        return PointResponse(False)
    else:

        updated_points = []
        deleted = False

        for p in points:

            if p['point_name'] == request.name:
                deleted = True
                continue

            updated_points.append(p)

        save_json(updated_points)

    if deleted:
        print("Point deleted")
    else:
        print("Error: The point not exists.")

    print(updated_points)
    print()
    return PointResponse(True)


def subcriber_callback(msg: Odometry):
    global robot_pose
    robot_pose = msg.pose.pose



if __name__ == "__main__":

    rospy.init_node('manager_goals')


    current_position = rospy.Service('/manager_goals/robot_current_position', Empty , current_position_callback)

    all_points = rospy.Service('/manager_goals/get_all_points', Empty , all_points_callback)
    add_new_point = rospy.Service('/manager_goals/add_new_point', Point , add_new_point_callback)
    delete_point = rospy.Service('/manager_goals/delete_point', Point , delete_point_callback)


    sub_pose = rospy.Subscriber('/odom_imu_encoder' , Odometry, subcriber_callback)

    rospy.spin()
