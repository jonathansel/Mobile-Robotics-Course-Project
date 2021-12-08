#!/usr/bin/env python
import rospy #ROS python functionality
import numpy as np
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf2_ros #Transform listener
import math
import time
from std_msgs.msg import UInt16, Int32 #Init UInt16 and UInt32 msg type
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal #Movebase Init

find_dog = False
find_plant = False

robot_trans_x = 0
robot_trans_y = 0
robot_rot_w = 0
robot_rot_z = 0

def cbFindObject(data):
    global find_dog, find_plant
    if data.data == 18:
        find_dog = True
        print("Driving to Dog")
        print(find_dog)
        movebase()

    if data.data == 64:
        find_plant = True
        print("Driving to Potted Plant")

def movebase():
    global find_dog
    global robot_trans_x, robot_trans_y, robot_rot_z, robot_rot_w
    if find_dog == True:
        client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        client.wait_for_server()

        print("transx:")
        print(robot_trans_x)

        print("transx:")
        print(robot_trans_y)

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = robot_trans_x 
        goal.target_pose.pose.position.y = robot_trans_y - 0.6
        goal.target_pose.pose.orientation.z = robot_rot_z
        goal.target_pose.pose.orientation.w = robot_rot_w
        rospy.loginfo("Sending Goal Back to Dog")
        client.send_goal(goal)
        wait = client.wait_for_result()
        find_dog = False #Set to false to ensure only completed once    

#Init ROS node and tf2 listener
rospy.init_node("object_find")
tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)

tfBufferPlant = tf2_ros.Buffer()
listenerPlant = tf2_ros.TransformListener(tfBufferPlant)

r = rospy.Rate(4)

sub_object = rospy.Subscriber('/object_finder', Int32, cbFindObject)

# While loop to transform robot base_footprint to map frame (tf2 listener)
while not rospy.is_shutdown():
    
    try:
        transformObject = tfBuffer.lookup_transform('map', 'dog', rospy.Time())
        trans = transformObject.transform.translation
        rot = transformObject.transform.rotation
        
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        r.sleep()
        continue    

    #Set global vars to found x,y position and quarternion rotation
    robot_trans_x = trans.x
    robot_trans_y = trans.y
    robot_rot_z = rot.z
    robot_rot_w = rot.w
    print(robot_trans_x)


    r.sleep()