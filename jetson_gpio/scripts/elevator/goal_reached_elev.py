#!/usr/bin/env python 
# -*- coding: utf-8 -*-
import rospy  
from actionlib_msgs.msg import *  
from geometry_msgs.msg import PoseWithCovarianceStamped,  PoseStamped, Pose, PoseArray, Twist
from move_base_msgs.msg import  MoveBaseActionResult, MoveBaseActionGoal, MoveBaseGoal 
from rospy.core import NullHandler  
from std_msgs.msg import Int64
import os
import sys


class Goal_reached():  
    def __init__(self):  
        rospy.init_node('goal_reached', anonymous=True)   
        self.call_elevator_sub = rospy.Subscriber('/call_ele_point',PoseStamped,self.elev_callback1) #呼叫电梯
        self.in_elevator_sub = rospy.Subscriber('/in_ele_point',PoseStamped,self.elev_callback2)     #进入电梯
        self.out_elevator_sub = rospy.Subscriber('/out_ele_point',PoseStamped,self.elev_callback3)   #退出电梯
        self.out_elevator_sub = rospy.Subscriber('/special_points',Int64,self.elev_callback4)   
        self.arrived_elev_pub = rospy.Publisher('/arrived_elev',Int64,queue_size=10)   #特征点发布的话题 内容为 date:1  date:2 date:3
        self.current_pose_sub= rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped,self.current_callback)
        self.waypoint_sub = rospy.Subscriber('/waypoints',PoseArray, self.goal_callback1)
        self.result_sub = rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.result_callback)
        self.goal_sub = rospy.Subscriber('/move_base/goal', MoveBaseActionGoal, self.goal_callback3)
        self.elevator_point_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped,queue_size=10)
        self.goal_pub = rospy.Publisher('/goal_back', PoseWithCovarianceStamped, queue_size=10)
        self.call_elev =0
        self.getstatusList=list()
        self.arrived_pose=0
        
    def elev_callback1(self,pose0):
        self.call_ele_point = pose0
        if pose0 != NullHandler:  #呼叫电梯点的话题不为空，发布点位信息，机器人前往该点
            self.elevator_point_pub.publish(self.call_ele_point)
            while not rospy.is_shutdown(): 
                if self.arrived_pose ==1 and self.call_elev ==1: #当机器人当前位置在目标位置范围内 AND status=3 发布到达点位状态信息
                    print(self.arrived_pose)
                    print("arrived P1")
                    self.data1=Int64()
                    self.data1.data=1
                    self.arrived_elev_pub.publish(self.data1)
                    self.call_elev =0
                    self.arrived_pose=0
                    break
    
    def elev_callback2(self,pose1):
        self.in_ele_point = pose1
        if pose1 != NullHandler: #电梯内点的话题不为空，发布点位信息，机器人前往该点
            while not rospy.is_shutdown(): 
                 if self.special_status ==2:
                    self.elevator_point_pub.publish(self.in_ele_point)
                    if self.arrived_pose ==1 and self.call_elev ==1:
                        print(self.arrived_pose)
                        print("arrived P2")
                        self.data1=Int64()
                        self.data1.data=2
                        self.arrived_elev_pub.publish(self.data1)
                        self.call_elev =0
                        self.arrived_pose=0
                        break
    
    def elev_callback3(self,pose2):
        self.out_ele_point = pose2
        if pose2 != NullHandler: #出电梯点的话题不为空，发布点位信息，机器人前往该点
            while not rospy.is_shutdown(): 
                if self.special_status ==3:
                    self.elevator_point_pub.publish(self.out_ele_point)
                    if self.arrived_pose ==1 and self.call_elev ==1:
                        print(self.arrived_pose)
                        print("arrived P3")
                        self.data1=Int64()
                        self.data1.data=3
                        self.arrived_elev_pub.publish(self.data1)
                        self.call_elev =0
                        self.arrived_pose=0
                        break
            
    def elev_callback4(self,msg2):
        self.special_status = msg2
        
            
    def current_callback(self,Pose):  #获取机器人当前位置的x y坐标
        self.current_pose_x = Pose.pose.pose.position.x
        self.current_pose_y = Pose.pose.pose.position.y
        print(self.current_pose_x)
        if self.goToX-0.2 < self.current_pose_x <=self.goToX +0.2  and  self.goToY-0.2 < self.current_pose_y <= self.goToY+0.2:
            self.arrived_pose=1
        else:
            self.arrived_pose=0
            
          
    def result_callback(self,msg):
        if msg.status.status ==3:   
            self.call_elev =1
            print(self.call_elev)
            if self.getEndPointX == self.goToX:
                self.goal = PoseWithCovarianceStamped()
                self.goal.pose.pose.position.x = self.getEndPointX
                self.goal.pose.pose.position.y = self.getEndPointY
                self.goal.pose.pose.orientation.z = self.getEndPointZ
                self.goal.pose.pose.orientation.w = self.getEndPointW
                self.goal_pub.publish(self.goal)
				#self.waypoint = []
				# #print(self.waypoint)
                self.getEndPoint = []		

    def goal_callback1(self,msg0):
        if msg0 != NullHandler :
            self.getList = msg0.poses
            self.getEndPoint = self.getList[-1]
            self.getEndPointX = self.getEndPoint.position.x
            self.getEndPointY = self.getEndPoint.position.y
            self.getEndPointZ = self.getEndPoint.orientation.z
            self.getEndPointW = self.getEndPoint.orientation.w
			#self.waypoint.append(self.getX)
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
