#!/usr/bin/env python3
#
# MOBILE ROBOTS - FI-UNAM, 2024-1
# RRT
#
# Instructions:
# Write the code necessary to plan a path using an
# occupancy grid and the A* algorithm
# MODIFY ONLY THE SECTIONS MARKED WITH THE 'TODO' COMMENT
#

import numpy
import heapq
import rospy
import math
from geometry_msgs.msg import PoseStamped, Pose, Point
from nav_msgs.msg import Path
from nav_msgs.srv import *
from collections import deque

NAME = "FULL NAME"

msg_path = Path()

class Node:
    def __init__(self, x, y):
        self.children = []
        self.parent = None
        self.x = x
        self.y = y

def get_random_q(grid_map):
    min_x = grid_map.info.origin.position.x
    min_y = grid_map.info.origin.position.y
    max_x = min_x + grid_map.info.width *grid_map.info.resolution
    max_y = min_y + grid_map.info.height*grid_map.info.resolution
    is_free = False
    attempts = len(grid_map.data)
    while not is_free and attempts > 0:
        x = numpy.random.uniform(min_x, max_x)
        y = numpy.random.uniform(min_y, max_y)
        c = int((x - min_x)/grid_map.info.resolution)
        r = int((y - min_y)/grid_map.info.resolution)
        is_free = grid_map.data[r*grid_map.info.width + c] < 40 and grid_map.data[r*grid_map.info.width + c] >= 0
        attempts -= 1
    return [x,y]

def get_nearest_node(tree, x, y):
    S = [tree] #Stack to traverse tree
    N = []     #List of nodes
    while len(S) > 0:
        n = S.pop()
        N.append(n)
        for c in n.children:
            S.append(c)
    distances = numpy.asarray([math.sqrt((x - n.x)**2 + (y - n.y)**2) for n in N])
    return N[numpy.argmin(distances)]


def rrt(start_x, start_y, goal_x, goal_y, grid_map):
    print("Calculating path by RRR from " + str([start_x, start_y])+" to "+str([goal_x, goal_y]))

    root = Node(start_x, start_y)
    [x,y] = get_random_q(grid_map)
    root.children.append(Node(x,y))
    [x,y] = get_random_q(grid_map)
    root.children.append(Node(x,y))
    n = get_nearest_node(root, 0.5, 0.5)
    print([n.x, n.y])
    
    #
    # TODO:
    # Review the RRT algorithm
    #
    return []

def get_inflated_map():
    print("Getting inflated map...")
    clt_inflated   = rospy.ServiceProxy("/inflated_map", GetMap)
    try:
        inflated_map = clt_inflated().map
        print("Using inflated map with " +str(len(inflated_map.data)) + " cells.")
    except:
        print("Cannot get inflated map. Terminating program...")
        exit()
    return inflated_map

def callback_rrt(req):
    grid_map = get_inflated_map()
    path = rrt(req.start.pose.position.x, req.start.pose.position.y,
               req.goal .pose.position.x, req.goal .pose.position.y, grid_map)
    msg_path.poses = []
    for [x,y] in path:
        msg_path.poses.append(PoseStamped(pose=Pose(position=Point(x=x, y=y))))
    return GetPlanResponse(msg_path)

def main():
    print("RRT - " + NAME)
    rospy.init_node("rrt")
    rospy.wait_for_service('/inflated_map')
    rospy.Service('/path_planning/a_star_search'  , GetPlan, callback_rrt)
    pub_path = rospy.Publisher('/path_planning/a_star_path', Path, queue_size=10)
    loop = rospy.Rate(2)
    msg_path.header.frame_id = "map"
    print("Inflated map service available")
    while not rospy.is_shutdown():
        pub_path.publish(msg_path)
        loop.sleep()

if __name__ == '__main__':
    main()
    
