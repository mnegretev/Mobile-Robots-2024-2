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
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist, PoseStamped, PointStamped
from trajectory_msgs.msg import JointTrajectory
from sound_play.msg import SoundRequest
from vision_msgs.srv import RecognizeObject, RecognizeObjectRequest
from manip_msgs.srv import InverseKinematicsPose2Traj, InverseKinematicsPose2TrajRequest, GetPolynomialTrajectory, GetPolynomialTrajectoryRequest
from hri_msgs.msg import RecognizedSpeech

NAME = "Roberto Ángel Camacho Bernabé"

#
# Global variable 'speech_recognized' contains the last recognized sentence
#
def callback_recognized_speech(msg):
    global recognized_speech, new_task, executing_task
    if executing_task:
        return
    new_task = True
    recognized_speech = msg.hypothesis[0]
    print("New command recognized: " + recognized_speech)

#
# Global variable 'goal_reached' is set True when the last sent navigation goal is reached
#
def callback_goal_reached(msg):
    global goal_reached
    goal_reached = msg.data
    print("Received goal reached: " + str(goal_reached))

def parse_command(cmd):
    obj = "pringles" if "PRINGLES" in cmd else "drink"
    loc = [8.0,8.5] if "TABLE" in cmd else [3.22, 9.72]
    return obj, loc

#
# Esta función envía la posición articular objetivo al brazo izquierdo y duerme 2 segundos
# para permitir que el brazo alcance la posición objetivo.
#
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

#
# Esta función envía una trayectoria articular al brazo izquierdo y duerme proporcionalmente
# a la longitud de la trayectoria para permitir que el brazo alcance la posición objetivo.
#
def move_left_arm_with_trajectory(Q):
    global pubLaGoalTraj
    pubLaGoalTraj.publish(Q)
    time.sleep(0.05*len(Q.points) + 2)

#
# Esta función envía la posición angular objetivo al gripper izquierdo y duerme 1 segundo
# para permitir que el gripper alcance el ángulo objetivo.
#
def move_left_gripper(q):
    global pubLaGoalGrip
    pubLaGoalGrip.publish(q)
    time.sleep(1.0)

#
# Esta función envía la posición articular objetivo al brazo derecho y duerme 2 segundos
# para permitir que el brazo alcance la posición objetivo.
#
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

#
# Esta función envía una trayectoria articular al brazo derecho y duerme proporcionalmente
# a la longitud de la trayectoria para permitir que el brazo alcance la posición objetivo.
#
def move_right_arm_with_trajectory(Q):
    global pubRaGoalTraj
    pubRaGoalTraj.publish(Q)
    time.sleep(0.05*len(Q.points) + 2)

#
# Esta función envía la posición angular objetivo al gripper derecho y duerme 1 segundo
# para permitir que el gripper alcance el ángulo objetivo.
#
def move_right_gripper(q):
    global pubRaGoalGrip
    pubRaGoalGrip.publish(q)
    time.sleep(1.0)

#
# Esta función envía la posición de inclinación y giro objetivo a la cabeza y duerme 1 segundo
# para permitir que la cabeza alcance la posición objetivo.
#
def move_head(pan, tilt):
    global pubHdGoalPose
    msg = Float64MultiArray()
    msg.data.append(pan)
    msg.data.append(tilt)
    pubHdGoalPose.publish(msg)
    time.sleep(1.0)

#
# Esta función envía una velocidad lineal y angular a la base móvil para realizar
# movimientos de bajo nivel. La base móvil se moverá a las velocidades lineal-angular
# dadas durante un tiempo 't'
#
def move_base(linear, angular, t):
    global pubCmdVel
    cmd = Twist()
    cmd.linear.x = linear
    cmd.angular.z = angular
    pubCmdVel.publish(cmd)
    time.sleep(t)
    pubCmdVel.publish(Twist())

#
# Esta función publica una posición de objetivo global. Este tema está suscrito por
# practice04 y realiza la planificación y seguimiento de caminos.
#
def go_to_goal_pose(goal_x, goal_y):
    global pubGoalPose
    goal_pose = PoseStamped()
    goal_pose.pose.orientation.w = 1.0
    goal_pose.pose.position.x = goal_x
    goal_pose.pose.position.y = goal_y
    pubGoalPose.publish(goal_pose)

#
# Esta función envía un texto a ser sintetizado.
#
def say(text):
    global pubSay
    msg = SoundRequest()
    msg.sound   = -3
    msg.command = 1
    msg.volume  = 1.0
    msg.arg2    = "voice_kal_diphone"
    msg.arg = text
    pubSay.publish(msg)

#
# Esta función llama al servicio para calcular la cinemática inversa del brazo izquierdo
# y devuelve la posición articular calculada.
#
def calculate_inverse_kinematics_left(x,y,z,roll, pitch, yaw):
    req_ik = InverseKinematicsPose2TrajRequest()
    req_ik.x = x
    req_ik.y = y
    req_ik.z = z
    req_ik.roll  = roll
    req_ik.pitch = pitch
    req_ik.yaw   = yaw
    req_ik.duration = 0
    req_ik.time_step = 0.05
    req_ik.initial_guess = []
    clt = rospy.ServiceProxy("/manipulation/la_ik_trajectory", InverseKinematicsPose2Traj)
    resp = clt(req_ik)
    return resp.articular_trajectory

#
# Esta función llama al servicio para calcular la cinemática inversa del brazo derecho
# y devuelve la posición articular calculada.
#
def calculate_inverse_kinematics_right(x,y,z,roll, pitch, yaw):
    req_ik = InverseKinematicsPose2TrajRequest()
    req_ik.x = x
    req_ik.y = y
    req_ik.z = z
    req_ik.roll  = roll
    req_ik.pitch = pitch
    req_ik.yaw   = yaw
    req_ik.duration = 0
    req_ik.time_step = 0.05
    req_ik.initial_guess = []
    clt = rospy.ServiceProxy("/manipulation/ra_ik_trajectory", InverseKinematicsPose2Traj)
    resp = clt(req_ik)
    return resp.articular_trajectory

#
# Llama al servicio para calcular una trayectoria polinómica para el brazo izquierdo
#
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
    return resp.trajectory

#
# Llama al servicio para calcular una trayectoria polinómica para el brazo derecho
#
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
    return resp.trajectory

#
# Llama al servicio para encontrar un objeto y devuelve
# las coordenadas xyz del objeto solicitado con respecto a "realsense_link"
#
def find_object(object_name):
    clt_find_object = rospy.ServiceProxy("/vision/obj_reco/detect_and_recognize_object", RecognizeObject)
    req_find_object = RecognizeObjectRequest()
    req_find_object.point_cloud = rospy.wait_for_message("/camera/depth_registered/points", PointCloud2)
    req_find_object.name  = object_name
    resp = clt_find_object(req_find_object)
    return [resp.recog_object.pose.position.x, resp.recog_object.pose.position.y, resp.recog_object.pose.position.z]

#
# Transforma un punto xyz expresado con respecto al marco fuente al marco objetivo
#
def transform_point(x,y,z, source_frame="realsense_link", target_frame="shoulders_left_link"):
    listener = tf.TransformListener()
    listener.waitForTransform(target_frame, source_frame, rospy.Time(), rospy.Duration(4.0))
    obj_p = PointStamped()
    obj_p.header.frame_id = source_frame
    obj_p.header.stamp = rospy.Time(0)
    obj_p.point.x, obj_p.point.y, obj_p.point.z = x,y,z
    obj_p = listener.transformPoint(target_frame, obj_p)
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
    pubCmdVel   = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    pubSay      = rospy.Publisher('/hri/speech_generator', SoundRequest, queue_size=10)
    pubLaGoalPose = rospy.Publisher("/hardware/left_arm/goal_pose" , Float64MultiArray, queue_size=10)
    pubRaGoalPose = rospy.Publisher("/hardware/right_arm/goal_pose", Float64MultiArray, queue_size=10)
    pubHdGoalPose = rospy.Publisher("/hardware/head/goal_pose"     , Float64MultiArray, queue_size=10)
    pubLaGoalGrip = rospy.Publisher("/hardware/left_arm/goal_gripper" , Float64, queue_size=10)
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

    #
    # FINAL PROJECT 
    #
    executing_task = False
    current_state = "SM_INIT"
    new_task = False
    goal_reached = False
    while not rospy.is_shutdown():
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

        #SM_MOVE_HEAD: Mueve la cabeza para localizar el objeto.
        elif current_state == "SM_MOVE_HEAD": 
            print("Moving head")
            move_head(0,-1)
            current_state = "SM_FIND_OBJECT"

        #SM_FIND_OBJECT: Encuentra la posición del objeto usando la visión.
        elif current_state == "SM_FIND_OBJECT":
            print("Trying to find: " + requested_object)
            x,y,z = find_object(requested_object)
            print("Found " + requested_object + "  at " + str([x,y,z]))
            current_state = "SM_TAKE_OBJECT"

        #SM_TAKE_OBJECT: Transforma las coordenadas del objeto, mueve el brazo apropiado
        #a la posición para agarrar el objeto, y cierra el gripper.
        elif current_state == "SM_TAKE_OBJECT":
            # Verificar el brazo adecuado
            target_frame = "shoulders_left_link" if requested_object == "pringles" else "shoulders_right_link"
            x, y, z = transform_point(x, y, z, target_frame=target_frame)
            print("Transformed position: " + str([x, y, z]))
            
            if requested_object == "pringles":
                # Lógica para el brazo izquierdo
                print("Moving left arm to initial position...")
                Q = get_la_polynomial_trajectory([-1, 0.2, 0, 1.55, 0, 1.16, 0], duration=4.0)
                move_left_arm_with_trajectory(Q)
                
                print("Opening left gripper")
                move_left_gripper(1.0)
                
                print("Moving left arm to position above the object...")
                intermediate_z = z + 0.2  # Subir el brazo 20 cm por encima del objeto
                Q = calculate_inverse_kinematics_left(x, y, intermediate_z, 0, -math.pi / 2, 0)
                move_left_arm_with_trajectory(Q)
                
                print("Lowering left arm to grasp the object...")
                Q.points[-1].positions[2] = z + 0.1  # Ajustar solo la coordenada z 10 cm por encima
                move_left_arm_with_trajectory(Q)
                
                rospy.loginfo("Closing left gripper to grasp the object")
                move_left_gripper(0.0)
                
            if requested_object == "drink":
                # Lógica para el brazo derecho
                print("Moving right arm to initial position...")
                Q = get_ra_polynomial_trajectory([-1, 0.2, 0, 1.55, 0, 1.16, 0], duration=4.0)
                move_right_arm_with_trajectory(Q)
                
                print("Opening right gripper")
                move_right_gripper(1.0)
                
                print("Moving right arm to position above the object...")
                intermediate_z = z + 0.2  # Subir el brazo 20 cm por encima del objeto
                Q = calculate_inverse_kinematics_right(x, y, intermediate_z, 0, -math.pi / 2, 0)
                move_right_arm_with_trajectory(Q)
                
                print("Lowering right arm to grasp the object...")
                Q.points[-1].positions[2] = z + 0.1  # Ajustar solo la coordenada z 10 cm por encima
                move_right_arm_with_trajectory(Q)
                
                rospy.loginfo("Closing right gripper to grasp the object")
                move_right_gripper(0.0)

            current_state = "SM_NAVIGATE_TO_LOCATION"

        #SM_NAVIGATE_TO_LOCATION: Navega hacia la ubicación solicitada.
        elif current_state == "SM_NAVIGATE_TO_LOCATION":
            rospy.loginfo("Navigating to " + str(requested_location))
            go_to_goal_pose(requested_location[0], requested_location[1])
            current_state = "SM_WAIT_GOAL_REACHED"

        #SM_WAIT_GOAL_REACHED: Espera hasta que el robot haya llegado a la ubicación solicitada.
        elif current_state == "SM_WAIT_GOAL_REACHED":
            if goal_reached:
                rospy.loginfo("Goal reached")
                current_state = "SM_PLACE_OBJECT"
                goal_reached = False

        #SM_PLACE_OBJECT: Coloca el objeto en la ubicación, abre el gripper y retrae el brazo.
        elif current_state == "SM_PLACE_OBJECT":
            if requested_object == "pringles":
                rospy.loginfo("Placing object at the destination")
                Q = get_la_polynomial_trajectory([-1, 0.2, 0, 1.55, 0, 1.16, 0], duration=4.0)
                move_left_arm_with_trajectory(Q)
                rospy.loginfo("Opening left gripper to release the object")
                move_left_gripper(1.0)
                rospy.loginfo("Retracting left arm")
                Q = get_la_polynomial_trajectory([0, 0.2, 0, 0.5, 0, 0.0, 0], duration=4.0)
                move_left_arm_with_trajectory(Q)
            if requested_object == "drink":
                rospy.loginfo("Placing object at the destination")
                Q = get_ra_polynomial_trajectory([-1, 0.2, 0, 1.55, 0, 1.16, 0], duration=4.0)
                move_right_arm_with_trajectory(Q)
                rospy.loginfo("Opening right gripper to release the object")
                move_right_gripper(1.0)
                rospy.loginfo("Retracting right arm")
                Q = get_ra_polynomial_trajectory([0, 0.2, 0, 0.5, 0, 0.0, 0], duration=4.0)
                move_right_arm_with_trajectory(Q)
            
            say("Task completed. The " + requested_object + " has been delivered.")
            current_state = "SM_TASK_COMPLETED"

        #SM_TASK_COMPLETED: Indica que la tarea ha sido completada y está esperando nuevas tareas.
        elif current_state == "SM_TASK_COMPLETED":
            rospy.loginfo("Task completed. Waiting for new tasks.")
            executing_task = False
            current_state = "SM_WAITING_NEW_TASK"

        loop.sleep()

if __name__ == '__main__':
    main()
