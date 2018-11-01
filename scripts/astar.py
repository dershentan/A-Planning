#!/usr/bin/env python

import sys
import rospy
import random
import math
import os
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point, Quaternion
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

global robot_position
robot_position = [-8.0, -2.0]
global roll, pitch, yaw
roll = pitch = yaw = 0.0
global goal_position, goalx, goaly
goalx = rospy.get_param('/astar/goalx')
goaly = rospy.get_param('/astar/goaly')
goal_position = [goalx, goaly]
global origin_position
origin_position = [8, 9]
global yaw_degrees
yaw_degrees = 0.0

with open('/home/robotics/catkin_ws/src/lab5/map/map.txt', 'r') as input:
    exec(input)
global map
map = np.array(map).reshape((20, 18))

def euclideanDistance(p1, p2):
      p1_x = p1[0]
      p1_y = p1[1]
      p2_x = p2[0]
      p2_y = p2[1]
      e_dist = math.sqrt(((p2_x - p1_x)**2) + ((p2_y - p1_y)**2))
      return round(e_dist, 1)
  
def nodeName(pX, pY):
    return str(pX) + ', ' + str(pY)

def aStarPlanning(p1, p2, pO,map):
    p1_x = int(pO[0]) + int(math.ceil(p1[0]))
    p1_y = int(pO[1]) - int(math.ceil(p1[1]))
    p2_x = int(pO[0]) + int(math.ceil(p2[0]))
    p2_y = int(pO[1]) - int(math.ceil(p2[1]))
    #print(map)
    #print('Start: ' + str(p1_x) + ', ' + str(p1_y))
    #print('End: ' + str(p2_x) + ', ' + str(p2_y))
    
    mapWidth = map.shape[1]
    mapLenght = map.shape[0]
    s_P = [p1_x, p1_y]
    e_P = [p2_x, p2_y]
    #print(str(s_P) + ' : ' + str(e_P))
    
    openNode = {}
    closedNode = {}
    g_n = euclideanDistance(s_P, s_P)
    h_n = euclideanDistance(s_P, e_P)
    f_n = 0.0 #starting node 
    # [own position, f(n), g(n), h(n), parent position] where f(n) = function cost(f(n)= g(n) + h(n)), ->
    #g(n) = cost of path(from start), h(n) = heuristic cost(dist from end)
    openNode[nodeName(p1_x, p1_y)] = [[p1_x, p1_y], f_n, g_n, h_n, None]
    
    while True:
        try:
             key_min_F = min(openNode.keys(), key=(lambda k: openNode[k][1]))
        except:
             print('--------No such path or goal position!--------')
             print('----Rerun program and try new coordinate!-----')
             sys.exit(0)
        currentNode = openNode[key_min_F]
        openNode.pop(key_min_F, None)
        closedNode[key_min_F] = currentNode
                  
        if currentNode[0] == [p2_x, p2_y]:
            #print('--------End--------')
            shortest_path = []
            shortest_path.append(currentNode[0])
            while not currentNode[0] == s_P:
                currentNode = closedNode[nodeName(currentNode[4][0], currentNode[4][1])]
                shortest_path.append(currentNode[0])
            return shortest_path
        
        currentNeighbours = []
        
        for i in range(currentNode[0][0]-1, currentNode[0][0]+2):
            for j in range(currentNode[0][1]-1, currentNode[0][1]+2):
                if 0 <= i < mapWidth:
                    if 0 <= j < mapLenght:
                        currentNeighbours.append([i, j])
        currentNeighbours.remove(currentNode[0])
        
        for neighbours in currentNeighbours:
            if map[neighbours[1], neighbours[0]] == 1 or nodeName(neighbours[0], neighbours[1]) in closedNode:
                continue
            elif not neighbours[0] == currentNode[0][0] and not neighbours[1] == currentNode[0][1]:
                if map[neighbours[1], currentNode[0][0]] == 1 and map[currentNode[0][1], neighbours[0]] == 1:
                    continue
            
            neighbour_gn = currentNode[1] + euclideanDistance(currentNode[0], neighbours)
            neighbour_hn = euclideanDistance(neighbours, e_P)
            neighbour_fn = neighbour_gn + neighbour_hn
            #print(str(neighbour_fn) + ' : ' + str(neighbour_gn) + ' : ' + str(neighbour_hn))
            
            if not nodeName(neighbours[0], neighbours[1]) in openNode or neighbour_fn < openNode[nodeName(neighbours[0], neighbours[1])][1]:
                openNode[nodeName(neighbours[0], neighbours[1])] = [neighbours, neighbour_fn, neighbour_gn, neighbour_hn, currentNode[0]]
                    
    return


def odometryCallback(msg):
      global robot_position, yaw_degrees
      global roll, pitch, yaw
      robot_position = [msg.pose.pose.position.x, msg.pose.pose.position.y]
      robot_orientation = [0.0, 0.0, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
      (roll, pitch, yaw) = euler_from_quaternion(robot_orientation)
      #print(robot_position)
      #print(str(roll) + ' ' + str(pitch) + ' ' + str(yaw))
      if yaw < 0:
           yaw = (2*math.pi) + yaw
           yaw_degrees = yaw * 180.0 / math.pi
      else:
           yaw_degrees = yaw * 180.0 / math.pi
      #print('yaw degrees raw: ' + str(yaw_degrees))
      if round(yaw_degrees, 1) == 360.0:
           yaw_degrees = 0.0
      #print('yaw degrees: ' + str(yaw_degrees))

def astar():
      rospy.init_node('astar', anonymous=True)
      rospy.Subscriber("/base_pose_ground_truth", Odometry, odometryCallback)
      robot0_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
      
      rate = rospy.Rate(10)
      
      global goal_position, goalx, goaly
      s_path = aStarPlanning(robot_position, goal_position, origin_position, map)
#      print(s_path)
#      for position in s_path:
#           map[position[1], position[0]] = 9
#      print(map)

      path_trace = []
      for s_p_it in s_path:
           path_trace.append([(s_p_it[0] - origin_position[0]), (origin_position[1] - s_p_it[1])])
      #if not path_trace[0] == goal_position:
           #path_trace[0] = goal_position
      print('Shortest Path: ')
      print(path_trace)

      cur_node = -1
      move_to_pt = [float(path_trace[cur_node][0]), float(path_trace[cur_node][1])]
      #move_to_pt = path_trace[cur_node]

      while not rospy.is_shutdown():
           twist = Twist()
           #print(math.sqrt( ((move_to_pt[1]-robot_position[1])**2) + ((move_to_pt[0]-robot_position[0])**2) ))

	   if not goal_position == [rospy.get_param('/astar/goalx'), rospy.get_param('/astar/goaly')]:
                print('Change of path: ')
                path_trace = []
                goalx = rospy.get_param('/astar/goalx')
                goaly = rospy.get_param('/astar/goaly')
                goal_position = [goalx, goaly]
                print(goal_position)
                s_path = aStarPlanning(robot_position, goal_position, origin_position, map)
                #print(s_path)
                #for position in s_path:
                #     map[position[1], position[0]] = 9
                #print(map)
                print('Current Position: '+ str(robot_position))
                for s_p_it in s_path:
                     path_trace.append([(s_p_it[0] - origin_position[0]), (origin_position[1] - s_p_it[1])])
                #if not path_trace[0] == goal_position:
                     #path_trace[0] = goal_position
                print('Shortest Path: ')
                print(path_trace)

                cur_node = -1
                move_to_pt = [float(path_trace[cur_node][0]), float(path_trace[cur_node][1])]
                #move_to_pt = path_trace[cur_node]


           #normal loop
           if not (round(goal_position[0], 1) == round(robot_position[0], 1) and round(goal_position[1], 1) == round(robot_position[1], 1)):
                #print(str(round(robot_position[0], 1)) + ' : ' + str(round(move_to_pt[0], 1)) + ' , ' + str(round(robot_position[1], 1)) + ' : ' + str(round(move_to_pt[1], 1)))
                dy = move_to_pt[1] - robot_position[1]
                dx = move_to_pt[0] - robot_position[0]
                atan2_angle = math.atan2(dy, dx)
                atan2_degrees = math.degrees(atan2_angle)
                #print('Pre-atan2 degrees: ' + str(atan2_degrees))
                if atan2_degrees < 0:
                     atan2_degrees += 360
                if round(atan2_degrees, 1) == 360.0:
                     atan2_degrees = 0.0
                #print('Compare degrees= ' + str(yaw_degrees) + ' : ' + str(atan2_degrees))
                #print('yaw degrees: ' + str(yaw_degrees))
           
                if not round(yaw_degrees, 1) == round(atan2_degrees, 1):
                     if -180.0 <= (yaw_degrees - atan2_degrees) <= 180.0:
                          if (yaw_degrees - atan2_degrees) < 0:
                               twist.angular.z = -((yaw_degrees - atan2_degrees) * math.pi / 180.0)
                               #print('1st: ' + str(-((yaw_degrees - atan2_degrees) * math.pi / 180.0)))
                          else:
                               twist.angular.z = -((yaw_degrees - atan2_degrees) * math.pi / 180.0)
                               #print('2nd: ' + str(-((yaw_degrees - atan2_degrees) * math.pi / 180.0)))
                     else:
                          if (atan2_degrees - yaw_degrees) < 0:
                               twist.angular.z = -((atan2_degrees - yaw_degrees) * math.pi / 180.0)
                               #print('3rd: ' + str(-((atan2_degrees - yaw_degrees) * math.pi / 180.0)))
                          else:
                               twist.angular.z = -((atan2_degrees - yaw_degrees) * math.pi / 180.0)
                               #print('4th: ' + str(-((atan2_degrees - yaw_degrees) * math.pi / 180.0)))
                     
                elif not ((round(robot_position[0], 1) == round(move_to_pt[0], 1)) and (round(robot_position[1], 1) == round(move_to_pt[1], 1))):
                     twist.linear.x = math.sqrt( ((move_to_pt[1]-robot_position[1])**2) + ((move_to_pt[0]-robot_position[0])**2) )
                     #print('Moving')
                
                robot0_pub.publish(twist)
           
           #update to new point in path
           if round(move_to_pt[0], 1) == round(robot_position[0], 1) and round(move_to_pt[1], 1) == round(robot_position[1], 1):
                if not cur_node == -len(path_trace):
                     cur_node -= 1
                     move_to_pt = [float(path_trace[cur_node][0]), float(path_trace[cur_node][1])]
                     #move_to_pt = path_trace[cur_node]
                else:
                     move_to_pt = [float(goal_position[0]), float(goal_position[1])]
           rate.sleep()

if __name__ == '__main__':
      astar()
