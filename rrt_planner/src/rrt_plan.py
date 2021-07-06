#!/usr/bin/env python
import rospy
from obstacle_detector import Obstacles
from geometry_msgs import PoseStamped
from ackermann_msgs import AckermannDriveStamped
from localmap_loader import PtsVector

import time
import math
import random
import matploblib.pyplot as plt
import numpy  as np

class RRT:
	
	class Node:
		def __init__(self, x, y):
			self.x = x
			self.y = y
			self.path_x = []
			self.path_y = []
			self.parent = None
	
	def __init__(self, start, goal, rand_area, expand_dist=1.0, path_resol=1.0, goal_sample_rate=1, max_iter=500):
		
		self.start = self.Node(start[0], start[1])
		self.end = self.Node(goal[0], goal[1])
		self.min_rand = rand_area[0]
		self.max_rand = rand_area[1]
		self.expand_dis = expand_dis
		self.path_resolution = path_resolution
		self.goal_sample_rate = goal_sample_rate
		self.max_iter = max_iter
		
		self.ld = 4.0
		self.goal_arr = []
		self.obs_arr = []
		
		rospy.Subscriber("center_pts", PtsVector, self.centerCallback)
		rospy.Subscriber("obstacle_pts", PtsVector, self.obsCallback)
		
		self.ack_pub = rospy.Publisher("/acker_msg", AckermannDriveStamped, queue_size=10)
		
	#############################
	#Goal & Obstacle Array Callback
	
	def centerCallback(self, ptsArray): #maybe not be used,,, we don't use delaunay
	 	for i in range(len(ptsArray)):
	 		self.goal_arr.append(ptsArray[i])
	 		
	 def obsCallback(self, ptsArray):
	 	for i in range(len(ptsArray)):
		 	self.obs_arr.append(ptsArray[i])	

	############################# 	
	
	def planning(self):
		self.tree = [self.start]
		
	
	def find_goal(self, goal_arr, ld):
		for i in range(len(goal_arr)):
			dist = math.hypot(goal_arr[i].pose.position.x,goal_arr[i].pose.position.y)
			if (dist > ld):
				goal = self.Node(goal_arr[i].pose.position.x, goal_arr[i].pose.position.y)
				break
		return goal, dist
		
	def main():
		rrt = RRT()
		path = rrt.planning()
		if path is None:
			print ("Cannot find Path")
		else:
			print("Found Path!")
		
if __name__ == '__main__':
	main()
		
		
		
		
		
		
		
		
		
		
		
		
		
	 	
	 	
	 	
	 	
	 	
	 	
	 	
