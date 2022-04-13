#!/usr/bin/env python 
# -*- coding: utf-8 -*-
import rospy  
from actionlib_msgs.msg import *  
import actionlib
from geometry_msgs.msg import PoseWithCovarianceStamped,  PoseStamped, Pose, PoseArray, Twist
from move_base_msgs.msg import  MoveBaseActionResult, MoveBaseAction, MoveBaseGoal 
from rospy.core import NullHandler  
from std_msgs.msg import Int64
import os
import sys


class Goal_reached():  
    def __init__(self):  
        rospy.init_node('goal_reached_elev', anonymous=True)
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(60))  
        rospy.loginfo("Connected to move base server")  
        
           
        self.call_elevator_sub = rospy.Subscriber('/call_ele_point',PoseStamped,self.elev_callback1) #呼叫电梯
        self.out_elevator_sub = rospy.Subscriber('/special_points',Int64,self.elev_callback4)   
        self.arrived_elev_pub = rospy.Publisher('/arrived_elev',Int64,queue_size=10)   #特征点发布的话题 内容为 date:1  date:2 date:3
        self.arrived_elev_sub = rospy.Subscriber('/arrived_elev',Int64, self.elev_callback2)
        self.current_pose_sub= rospy.Subscriber("/tracked_pose", PoseStamped,self.current_callback)
        self.result_sub = rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.result_callback)
        self.elevator_point_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped,queue_size=10)
        self.goal_pub = rospy.Publisher('/goal_back', PoseWithCovarianceStamped, queue_size=10)
        self.call_elev =0
        self.getstatusList=list()
        self.arrived_pose=0
        
        self.in_ele_point = MoveBaseGoal()
        self.in_ele_point.target_pose.pose.position.x = -0.8255543776220731
        self.in_ele_point.target_pose.pose.position.y = -0.11118848660122817
        self.in_ele_point.target_pose.pose.orientation.z = -0.7214382707978507
        self.in_ele_point.target_pose.pose.orientation.w = 0.6924787516077351
        
        self.out_ele_point = MoveBaseGoal() 
        self.out_ele_point.target_pose.pose.position.x = -0.8832419046382717
        self.out_ele_point.target_pose.pose.position.y = -1.612995809735922
        self.out_ele_point.target_pose.pose.orientation.z = -0.9998462295852648
        self.out_ele_point.target_pose.pose.orientation.w = -0.017536167886112675
        
        self.goal_lists=[self.in_ele_point, self.out_ele_point]
        
        
    def elev_callback1(self,pose0):
        self.call_ele_point = PoseStamped()
        self.call_ele_point.header.frame_id = 'map'  
        self.call_ele_point.header.stamp = rospy.Time.now()
        self.call_ele_point.pose.position.x = pose0.pose.position.x
        self.call_ele_point.pose.position.y = pose0.pose.position.y
        self.call_ele_point.pose.orientation.z = pose0.pose.orientation.z
        self.call_ele_point.pose.orientation.w = pose0.pose.orientation.w
        self.elevator_point_pub.publish(self.call_ele_point)
        self.elevator_X = pose0.pose.position.x
        self.elevator_Y = pose0.pose.position.y
        #pose0=NullHandler
        while not rospy.is_shutdown():
            if self.arrived_pose ==1:
                if self.call_elev ==1: #当机器人当前位置在目标位置范围内 AND status=3 发布到达点位状态信息
                    print(self.arrived_pose)
                    print("arrived P1")
                    self.data1=Int64()
                    self.data1.data=1
                    self.arrived_elev_pub.publish(self.data1)
                    self.call_elev =0
                    self.arrived_pose=0
                    break
    
    def elev_callback2(self,msg1):
        while not rospy.is_shutdown(): 
            if msg1 == 1:
                if self.special_status ==2:
                    self.move_base.send_goal( self.goal_lists[0])
                    if self.arrived_pose ==1 and self.call_elev ==1:
                        print(self.arrived_pose)
                        print("arrived P2")
                        self.data1=Int64()
                        self.data1.data=2
                        self.arrived_elev_pub.publish(self.data1)
                        self.call_elev =0
                        self.arrived_pose=0
            elif msg1 == 2:
                if self.special_status ==3:
                    self.move_base.send_goal( self.goal_lists[1])
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
        self.current_pose_x = Pose.pose.position.x
        self.current_pose_y = Pose.pose.position.y
        print(self.current_pose_x)
        if self.elevator_X-0.2 < self.current_pose_x <=self.elevator_X +0.2  and  self.elevator_Y-0.2 < self.current_pose_y <= self.elevator_Y+0.2:
            self.arrived_pose=1
        else:
            self.arrived_pose=0
            
    def result_callback(self,msg):
        if msg.status.status ==3:   
            self.call_elev =1
            
            

if __name__ == '__main__':  
    try:  
        Goal_reached()  
        rospy.spin()  

    except rospy.ROSInterruptException:  
        rospy.loginfo("Exploring SLAM finished.")
