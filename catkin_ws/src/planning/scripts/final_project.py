#!/usr/bin/env python3
#
# MOBILE ROBOTS - FI-UNAM, 2024-2
# FINAL PROJECT - SIMPLE SERVICE ROBOT
# 
# Instructions:
# Write the code necessary to make the robot to perform the following possible commands:
# * Robot take the <pringles|drink> to the <table|kitchen>
# You can choose where the table and kitchen are located within the map.
# The Robot must recognize the orders using speech recognition.
# Entering the command by text or similar way is not allowed.
# The Robot must announce the progress of the action using speech synthesis,
# for example: I'm going to grab..., I'm going to navigate to ..., I arrived to..., etc.
# Publishers and suscribers to interact with the subsystems (navigation,
# vision, manipulation, speech synthesis and recognition) are already declared. 
#
import rospy
import tf
import math
import time
from std_msgs.msg import String, Float64MultiArray, Float64, Bool
from nav_msgs.msg import Path
from nav_msgs.srv import GetPlan, GetPlanRequest
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist, PoseStamped, Pose, Point, PointStamped
from trajectory_msgs.msg import JointTrajectory
from sound_play.msg import SoundRequest
from vision_msgs.srv import *
from manip_msgs.srv import *
from hri_msgs.msg import *

NAME = " BAFO SEMICONDUCTORS INC. "

# Global variables
recognized_speech = ""
new_task = False
executing_task = False
goal_reached = False

def callback_recognized_speech(msg):
    global recognized_speech, new_task, executing_task
    if executing_task:
        return
    new_task = True
    recognized_speech = msg.hypothesis[0]
    print("New command recognized: " + recognized_speech)

def callback_goal_reached(msg):
    global goal_reached
    goal_reached = msg.data
    print("Received goal reached: " + str(goal_reached))

def parse_command(cmd):
    obj = "pringles" if "pringles" in cmd.lower() else "drink"
    loc = [8.0, 8.5] if "table" in cmd.lower() else [3.22, 9.72]
    return obj, loc

def move_left_arm(q1,q2,q3,q4,q5,q6,q7):
    global pubLaGoalPose
    msg = Float64MultiArray()
    msg.data.append(q1)
    msg.data.append(q2)
    msg.data.append(q3)
    msg.data.append(q4)
    msg.data.append(q5)
    msg.data.append(q6)
    msg.data.append(q7)
    pubLaGoalPose.publish(msg)
    time.sleep(2.0)

def move_left_arm_with_trajectory(Q):
    global pubLaGoalTraj
    if not isinstance(Q, JointTrajectory):
        print("Invalid trajectory format")
        return
    if len(Q.points) == 0:
        print("Empty trajectory")
        return
    print(f"Publishing trajectory with {len(Q.points)} points")
    pubLaGoalTraj.publish(Q)
    rospy.sleep(0.05 * len(Q.points) + 2)
    print("Trajectory execution completed")

def move_left_gripper(q):
    global pubLaGoalGrip
    pubLaGoalGrip.publish(q)
    print(f"Moving left gripper to: {q}")
    time.sleep(1.0)

def move_right_arm(q1,q2,q3,q4,q5,q6,q7):
    global pubRaGoalPose
    msg = Float64MultiArray()
    msg.data.append(q1)
    msg.data.append(q2)
    msg.data.append(q3)
    msg.data.append(q4)
    msg.data.append(q5)
    msg.data.append(q6)
    msg.data.append(q7)
    pubRaGoalPose.publish(msg)
    time.sleep(2.0)


def move_right_arm_with_trajectory(Q):
    global pubRaGoalTraj
    pubRaGoalTraj.publish(Q)
    print(f"Moving right arm with trajectory: {Q}")
    time.sleep(0.05 * len(Q.points) + 2)

def move_right_gripper(q):
    global pubRaGoalGrip
    pubRaGoalGrip.publish(q)
    print(f"Moving right gripper to: {q}")
    time.sleep(1.0)

def move_head(pan, tilt):
    global pubHdGoalPose
    msg = Float64MultiArray()
    msg.data = [pan, tilt]
    pubHdGoalPose.publish(msg)
    print(f"Moving head to pan: {pan}, tilt: {tilt}")
    time.sleep(1.0)

def move_base(linear, angular, t):
    global pubCmdVel
    cmd = Twist()
    cmd.linear.x = linear
    cmd.angular.z = angular
    pubCmdVel.publish(cmd)
    print(f"Moving base with linear: {linear}, angular: {angular}, time: {t}")
    time.sleep(t)
    pubCmdVel.publish(Twist())

def go_to_goal_pose(goal_x, goal_y):
    global pubGoalPose
    goal_pose = PoseStamped()
    goal_pose.pose.orientation.w = 1.0
    goal_pose.pose.position.x = goal_x
    goal_pose.pose.position.y = goal_y
    pubGoalPose.publish(goal_pose)
    print(f"Going to goal pose: x: {goal_x}, y: {goal_y}")

def say(text):
    global pubSay
    msg = SoundRequest()
    msg.sound = -3
    msg.command = 1
    msg.volume = 1.0
    msg.arg2 = "voice_kal_diphone"
    msg.arg = text
    pubSay.publish(msg)
    print(f"Saying: {text}")

def calculate_inverse_kinematics_left(x, y, z, roll, pitch, yaw):
    req_ik = InverseKinematicsPose2TrajRequest()
    req_ik.x = x
    req_ik.y = y
    req_ik.z = z
    req_ik.roll = roll
    req_ik.pitch = pitch
    req_ik.yaw = yaw
    req_ik.duration = 0
    req_ik.time_step = 0.05
    req_ik.initial_guess = []
    clt = rospy.ServiceProxy("/manipulation/la_ik_trajectory", InverseKinematicsPose2Traj)
    resp = clt(req_ik)
    print(f"Calculated IK for left arm: {resp.articular_trajectory}")
    return resp.articular_trajectory

def calculate_inverse_kinematics_right(x, y, z, roll, pitch, yaw):
    req_ik = InverseKinematicsPose2TrajRequest()
    req_ik.x = x
    req_ik.y = y
    req_ik.z = z
    req_ik.roll = roll
    req_ik.pitch = pitch
    req_ik.yaw = yaw
    req_ik.duration = 0
    req_ik.time_step = 0.05
    req_ik.initial_guess = []
    clt = rospy.ServiceProxy("/manipulation/ra_ik_trajectory", InverseKinematicsPose2Traj)
    resp = clt(req_ik)
    print(f"Calculated IK for right arm: {resp.articular_trajectory}")
    return resp.articular_trajectory

def get_la_polynomial_trajectory(q, duration=2.0, time_step=0.05):
    current_p = rospy.wait_for_message("/hardware/left_arm/current_pose", Float64MultiArray)
    current_p = current_p.data
    clt = rospy.ServiceProxy("/manipulation/polynomial_trajectory", GetPolynomialTrajectory)
    req = GetPolynomialTrajectoryRequest()
    req.p1 = current_p
    req.p2 = q
    req.duration = duration
    req.time_step = time_step
    resp = clt(req)
    print(f"Generated polynomial trajectory for left arm: {resp.trajectory}")
    return resp.trajectory

def get_ra_polynomial_trajectory(q, duration=5.0, time_step=0.05):
    current_p = rospy.wait_for_message("/hardware/right_arm/current_pose", Float64MultiArray)
    current_p = current_p.data
    clt = rospy.ServiceProxy("/manipulation/polynomial_trajectory", GetPolynomialTrajectory)
    req = GetPolynomialTrajectoryRequest()
    req.p1 = current_p
    req.p2 = q
    req.duration = duration
    req.time_step = time_step
    resp = clt(req)
    print(f"Generated polynomial trajectory for right arm: {resp.trajectory}")
    return resp.trajectory

def find_object(object_name):
    clt_find_object = rospy.ServiceProxy("/vision/obj_reco/detect_and_recognize_object", RecognizeObject)
    req_find_object = RecognizeObjectRequest()
    req_find_object.point_cloud = rospy.wait_for_message("/camera/depth_registered/points", PointCloud2)
    req_find_object.name = object_name
    resp = clt_find_object(req_find_object)
    print(f"Found object {object_name} at: {resp.recog_object.pose.position}")
    return [resp.recog_object.pose.position.x, resp.recog_object.pose.position.y, resp.recog_object.pose.position.z]

def transform_point(x, y, z, source_frame="realsense_link", target_frame="shoulders_left_link"):
    listener = tf.TransformListener()
    listener.waitForTransform(target_frame, source_frame, rospy.Time(), rospy.Duration(4.0))
    obj_p = PointStamped()
    obj_p.header.frame_id = source_frame
    obj_p.header.stamp = rospy.Time(0)
    obj_p.point.x, obj_p.point.y, obj_p.point.z = x, y, z
    obj_p = listener.transformPoint(target_frame, obj_p)
    print(f"Transformed point to {target_frame}: {obj_p.point}")
    return [obj_p.point.x, obj_p.point.y, obj_p.point.z]

def main():
    global new_task, recognized_speech, executing_task, goal_reached
    global pubLaGoalPose, pubRaGoalPose, pubHdGoalPose, pubLaGoalGrip, pubRaGoalGrip
    global pubLaGoalTraj, pubRaGoalTraj, pubGoalPose, pubCmdVel, pubSay
    print("FINAL PROJECT - " + NAME)
    rospy.init_node("final_project")
    rospy.Subscriber('/hri/sp_rec/recognized', RecognizedSpeech, callback_recognized_speech)
    rospy.Subscriber('/navigation/goal_reached', Bool, callback_goal_reached)
    pubGoalPose = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    pubCmdVel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    pubSay = rospy.Publisher('/hri/speech_generator', SoundRequest, queue_size=10)
    pubLaGoalPose = rospy.Publisher("/hardware/left_arm/goal_pose", Float64MultiArray, queue_size=10)
    pubRaGoalPose = rospy.Publisher("/hardware/right_arm/goal_pose", Float64MultiArray, queue_size=10)
    pubHdGoalPose = rospy.Publisher("/hardware/head/goal_pose", Float64MultiArray, queue_size=10)
    pubLaGoalGrip = rospy.Publisher("/hardware/left_arm/goal_gripper", Float64, queue_size=10)
    pubRaGoalGrip = rospy.Publisher("/hardware/right_arm/goal_gripper", Float64, queue_size=10)
    pubLaGoalTraj = rospy.Publisher("/manipulation/la_q_trajectory", JointTrajectory, queue_size=10)
    pubRaGoalTraj = rospy.Publisher("/manipulation/ra_q_trajectory", JointTrajectory, queue_size=10)
    listener = tf.TransformListener()
    loop = rospy.Rate(10)
    print("Waiting for services...")
    rospy.wait_for_service('/manipulation/la_ik_pose')
    rospy.wait_for_service('/manipulation/ra_ik_pose')
    rospy.wait_for_service('/vision/obj_reco/detect_and_recognize_object')
    print("Services are now available.")

    executing_task = False
    current_state = "SM_INIT"
    new_task = False
    goal_reached = False
    while not rospy.is_shutdown():
        print(f"Current state: {current_state}")  
        if current_state == "SM_INIT":
            print("Waiting for new task")
            current_state = "SM_WAITING_NEW_TASK"
        elif current_state == "SM_WAITING_NEW_TASK":
            if new_task:
                requested_object, requested_location = parse_command(recognized_speech)
                print("New task received: " + requested_object + " to  " + str(requested_location))
                say("Executing the command, " + recognized_speech)
                current_state = "SM_MOVE_HEAD"
                new_task = False
                executing_task = True
        elif current_state == "SM_MOVE_HEAD":
            print("Moving head to look for the object")
            move_head(0, -0.5) 
            say("Looking for the " + requested_object)
            current_state = "SM_FIND_OBJECT"
        elif current_state == "SM_FIND_OBJECT":
            try:
                print(f"Finding object: {requested_object}")
                obj_pos = find_object(requested_object)
                say("I found the " + requested_object)
                current_state = "SM_MOVE_ARM"
            except Exception as e:
                print(f"Error finding object: {e}")
                say("I couldn't find the " + requested_object)
                current_state = "SM_TASK_FAILED"
        elif current_state == "SM_MOVE_ARM":
            print("Moving arm to pick up the object")
            x, y, z = obj_pos
            target_position = transform_point(x, y, z)
            traj = calculate_inverse_kinematics_left(*target_position, 0, math.pi/2, 0)
            move_left_arm_with_trajectory(traj)
            move_left_gripper(0.8)
            say("I am picking up the " + requested_object)
            current_state = "SM_NAVIGATE"
        elif current_state == "SM_NAVIGATE":
            goal_x, goal_y = requested_location
            say("I am going to the " + ("table" if "table" in recognized_speech.lower() else "kitchen"))
            go_to_goal_pose(goal_x, goal_y)
            current_state = "SM_WAIT_GOAL_REACHED"
        elif current_state == "SM_WAIT_GOAL_REACHED":
            if goal_reached:
                say("I have arrived at the " + ("table" if "table" in recognized_speech.lower() else "kitchen"))
                current_state = "SM_PLACE_OBJECT"
        elif current_state == "SM_PLACE_OBJECT":
            print("Placing object")
            move_left_gripper(0)
            say("I have placed the " + requested_object)
            current_state = "SM_TASK_COMPLETED"
        elif current_state == "SM_TASK_COMPLETED":
            print("Task completed")
            executing_task = False
            current_state = "SM_WAITING_NEW_TASK"
            say("Task completed")
        elif current_state == "SM_TASK_FAILED":
            print("Task failed")
            executing_task = False
            current_state = "SM_WAITING_NEW_TASK"
            say("Task failed, please give a new command")

        loop.sleep()

if __name__ == '__main__':
    main()
