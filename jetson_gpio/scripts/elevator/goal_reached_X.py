#!/usr/bin/env python 
# -*- coding: utf-8 -*-
import rospy  
from actionlib_msgs.msg import *  
from geometry_msgs.msg import PoseWithCovarianceStamped,  PoseStamped, Pose, PoseArray
from move_base_msgs.msg import  MoveBaseActionResult, MoveBaseActionGoal, MoveBaseGoal 
from rospy.core import NullHandler  
import os
import sys

class Goal_reached():  
	def __init__(self):  
		rospy.init_node('goal_reached', anonymous=True)   
		self.waypoint_sub = rospy.Subscriber('/waypoints',PoseArray, self.goal_callback1)
		self.result_sub = rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.result_callback)
		self.goal_sub = rospy.Subscriber('/move_base/goal', MoveBaseActionGoal, self.goal_callback3)
		#self.simple_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback2)
		self.goal_pub = rospy.Publisher('/goal_back', PoseWithCovarianceStamped, queue_size=10)

	def result_callback(self,msg):
		if msg.status.status ==3:
			if self.getEndPointX == self.goToX:
				self.goal = PoseWithCovarianceStamped()  
				self.goal.pose.pose.position.x = self.getEndPointX
				self.goal.pose.pose.position.y = self.getEndPointY
				self.goal.pose.pose.orientation.z = self.getEndPointZ
				self.goal.pose.pose.orientation.w = self.getEndPointW
				self.goal_pub.publish(self.goal)
				#self.waypoint = []
				#print(self.waypoint)
				self.getEndPoint = []


	#向/goal_back发布消息后，/waypoints仍会持续接收消息，所以会出错，但不影响程序使用
	def goal_callback1(self,msg0):
		if msg0 != NullHandler :
			self.getList = msg0.poses
			self.getEndPoint = self.getList[-1]
			self.getEndPointX = self.getEndPoint.position.x
			self.getEndPointY = self.getEndPoint.position.y
			self.getEndPointZ = self.getEndPoint.orientation.z
			self.getEndPointW = self.getEndPoint.orientation.w
			print("getEndPointX: "+str(self.getEndPointX))
			#print(type(self.getEndPointX))



	def goal_callback3(self,msg1):
		if msg1 != NullHandler :
			self.goToX = msg1.goal.target_pose.pose.position.x
			self.goToY = msg1.goal.target_pose.pose.position.y
			self.goToZ = msg1.goal.target_pose.pose.orientation.z
			self.goToW = msg1.goal.target_pose.pose.orientation.w
			print("goToX: "+str(self.goToX))

            
            

if __name__ == '__main__':  
    try:  
        Goal_reached()  
        rospy.spin()  

    except rospy.ROSInterruptException:  
        rospy.loginfo("Exploring SLAM finished.")
