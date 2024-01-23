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
from visualization_msgs.msg import Marker
from nav_msgs.msg import Path
from nav_msgs.srv import *
from collections import deque

NAME = "FULL NAME"

class Node:
    def __init__(self, x, y, parent=None):
        self.children = []
        self.parent = parent
        self.x = x
        self.y = y

def in_free_space(x,y,grid_map):
    c = int((x - grid_map.info.origin.position.x)/grid_map.info.resolution)
    r = int((y - grid_map.info.origin.position.y)/grid_map.info.resolution)
    return grid_map.data[r*grid_map.info.width + c] < 40 and grid_map.data[r*grid_map.info.width + c] >= 0

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
        is_free = in_free_space(x,y,grid_map)
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

def get_new_node(nearest_node, rnd_x, rnd_y, epsilon):
    dist = math.sqrt((nearest_node.x - rnd_x)**2 + (nearest_node.y - rnd_y)**2)
    mag = min(dist, epsilon)
    if (dist == 0):
        return None
    return Node(nearest_node.x + mag*(rnd_x - nearest_node.x)/dist, 
                nearest_node.y + mag*(rnd_y - nearest_node.y)/dist, nearest_node)

def check_collision(x1, y1, x2, y2, grid_map):
    n = 2*int(max(abs(x2-x1), abs(y2-y1))/grid_map.info.resolution)
    P = numpy.linspace([x1,y1], [x2,y2], n)
    for x,y in P:
        if not in_free_space(x,y,grid_map):
            return True
    return False


def rrt(start_x, start_y, goal_x, goal_y, grid_map, epsilon):
    print("Calculating path by RRR from " + str([start_x, start_y])+" to "+str([goal_x, goal_y]))
    steps = 0
    
    tree = Node(start_x, start_y)
    goal_reached = False
    while not goal_reached and steps < 10000:
        [x,y] = get_random_q(grid_map)
        nearest_node = get_nearest_node(tree, x, y)
        new_node     = get_new_node(nearest_node, x, y, epsilon)
        if not check_collision(nearest_node.x, nearest_node.y, new_node.x, new_node.y, grid_map):
            nearest_node.children.append(new_node)
            if not check_collision(new_node.x, new_node.y, goal_x, goal_y, grid_map):
                goal_node = Node(goal_x, goal_y, new_node)
                new_node.children.append(goal_node)
                goal_reached = True
            steps += 1
    #
    # TODO:
    # Review the RRT algorithm
    #
    path = []
    if goal_reached:
        print("Path planned using RRT after " + str(steps) + " steps")
        while goal_node.parent is not None:
            path.insert(0, [goal_node.x, goal_node.y])
            goal_node = goal_node.parent
    else:
        print("Cannot plan path after " + str(steps) + " steps")
    return tree, path

def get_tree_marker(tree):
    mrk = Marker()
    mrk.header.stamp = rospy.Time.now()
    mrk.header.frame_id = "map"
    mrk.ns = "path_planning"
    mrk.id = 0
    mrk.type   = Marker.LINE_LIST
    mrk.action = Marker.ADD
    mrk.color.r = 0.3
    mrk.color.g = 0.3
    mrk.color.b = 1.0
    mrk.color.a = 0.7
    mrk.scale.x = 0.03
    mrk.pose.orientation.w = 1.0
    S = [tree] #Stack to traverse tree
    while len(S) > 0:
        n = S.pop()
        for c in n.children:
            mrk.points.append(Point(x=n.x, y=n.y, z=0))
            mrk.points.append(Point(x=c.x, y=c.y, z=0))
            S.append(c)
    return mrk

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
    global msg_path, msg_tree
    grid_map = get_inflated_map()
    tree,path = rrt(req.start.pose.position.x, req.start.pose.position.y,
                    req.goal .pose.position.x, req.goal .pose.position.y, grid_map, 0.5)

    msg_tree = get_tree_marker(tree)
    msg_path.poses = []
    for [x,y] in path:
        msg_path.poses.append(PoseStamped(pose=Pose(position=Point(x=x, y=y))))
    print("Sending path in service response")
    return GetPlanResponse(msg_path)

def main():
    global msg_path, msg_tree, epsilon
    print("RRT - " + NAME)
    rospy.init_node("rrt")
    rospy.wait_for_service('/inflated_map')
    rospy.Service('/path_planning/rrt_search'  , GetPlan, callback_rrt)
    pub_path = rospy.Publisher('/path_planning/rrt_path', Path, queue_size=10)
    pub_tree = rospy.Publisher('/path_planning/rrt_tree', Marker, queue_size=10)
    loop = rospy.Rate(2)
    msg_path = Path()
    msg_tree = Marker()
    msg_path.header.frame_id = "map"
    msg_tree = get_tree_marker(Node(0,0))
    epsilon = 0.5
    print("Inflated map service available")
    while not rospy.is_shutdown():
        pub_path.publish(msg_path)
        pub_tree.publish(msg_tree)
        loop.sleep()

if __name__ == '__main__':
    main()
    
