#!/usr/bin/env python 
# -*- coding: utf-8 -*-
import rospy  
from actionlib_msgs.msg import *  
from geometry_msgs.msg import PoseWithCovarianceStamped,  PoseStamped, Pose, PoseArray
from move_base_msgs.msg import  MoveBaseActionResult, MoveBaseActionGoal, MoveBaseGoal 
from rospy.core import NullHandler  
from std_msgs.msg import String,Int64
import os
import sys
import time
import threading

class Goal_reached():  
	def __init__(self):  
		#rospy.init_node('goal_reached', anonymous=True)   
		#self.waypoint_sub = rospy.Subscriber('/waypoint_mark',PoseWithCovarianceStamped, self.goal_callback1)
		#self.result_sub = rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.result_callback)
		#self.goal_sub = rospy.Subscriber('/move_base/goal', MoveBaseActionGoal, self.goal_callback3)
		#self.simple_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback2)
		#self.goal_pub = rospy.Publisher('/goal_back', Int64, queue_size=10)
		#self.goal = PoseWithCovarianceStamped()  
		#self.getList = []
		#self.getEndPoint = Pose()
		#self.goToX = 1.0
		#self.getEndPointX = 1.0
		self.init()

	def init(self):
		self.listener()

	def listener(self):
		# 建立节点
		rospy.init_node('goal_reached', anonymous=True)   
		# 订阅话题
		self.waypoint_sub = rospy.Subscriber('/waypoint_mark',PoseWithCovarianceStamped, self.goal_callback1)
		self.result_sub = rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.result_callback)
		self.goal_sub = rospy.Subscriber('/move_base/goal', MoveBaseActionGoal, self.goal_callback3)
		#发布话题
		self.goal_pub = rospy.Publisher('/goal_back', Int64, queue_size=10)
		# 运行线程1,收话题线程
		t1 = threading.Thread(target=self.thread_spin)  # 末端位置订阅线程
		t1.start()
		#处理其他函数
		while not rospy.is_shutdown():
			#处理其他函数
			pass

	def thread_spin(self):
		rospy.spin()


	def start_thread(self):

		wait_waypoint_mark = threading.Thread(target=self.wait_waypoint_mark)
		wait_waypoint_mark.start()

	def result_callback(self,msg):
		if msg.status.status ==3:
			if self.getEndPointX == self.goToX:
				#self.goal.pose.pose.position.x = self.getEndPointX
				#self.goal.pose.pose.position.y = self.getEndPointY
				#self.goal.pose.pose.orientation.z = self.getEndPointZ
				#self.goal.pose.pose.orientation.w = self.getEndPointW
				print("iiiiiii")
				#self.goal_pub.publish(1)
				#self.waypoint = []
				#print(self.waypoint)
				#print(id(self.goal),id(self.getList),id(self.getEndPoint),id(self.goToX),id(self.getEndPointX))
				#del self.getList[:]

	#向/goal_back发布消息后，/waypoints仍会持续接收消息，所以会出错，但不影响程序使用
	def wait_waypoint_mark(self):
		"""thread worker function"""
		data = rospy.wait_for_message('/waypoint_mark', PoseWithCovarianceStamped, timeout=None)
		rospy.loginfo('Recieved waypoint_mark message')
	

	def goal_callback1(self,msg0):
		#if msg0 != NullHandler :
		self.getEndPointX = msg0.pose.pose.position.x
			#print(str(self.getList))
			#print(type(self.getList)) #list
			#self.getEndPoint = self.getList[-1]
			#print(type(self.getEndPoint))  #<class 'geometry_msgs.msg._Pose.Pose'>
			#self.getEndPointX = self.getEndPoint.position.x
			#self.getEndPointY = self.getEndPoint.position.y
			#self.getEndPointZ = self.getEndPoint.orientation.z
			#self.getEndPointW = self.getEndPoint.orientation.w
			#self.waypoint.append(self.getX)
		print("getEndPointX: "+str(self.getEndPointX))
			#print(id(self.getList),id(self.getEndPoint),id(self.getEndPointX))
			#float
			#print(type(self.getEndPointX))

	def goal_callback3(self,msg1):
		if msg1 != NullHandler :
			self.goToX = msg1.goal.target_pose.pose.position.x
			#self.goToY = msg1.goal.target_pose.pose.position.y
			#self.goToZ = msg1.goal.target_pose.pose.orientation.z
			#self.goToW = msg1.goal.target_pose.pose.orientation.w
			print("goToX: "+str(self.goToX))
			print("getEndPointX: "+str(self.getEndPointX))
			#print(id(self.goToX))
			#float
	#程序重启代码 没用到
	#def restart_program():
		#python = sys.executable
		#os.execl(python, python, * sys.argv)

if __name__ == '__main__':  
    #try:  
        #print("end in 3s later.......")
        #time.sleep(3)
        #restart_program()
        Goal_reached()  
        rospy.spin()  

    #except rospy.ROSInterruptException:  
        #rospy.loginfo("Exploring SLAM finished.")
