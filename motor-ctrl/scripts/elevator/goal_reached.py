#!/usr/bin/env python 
# -*- coding: utf-8 -*-
import rospy  
from actionlib_msgs.msg import *  
from geometry_msgs.msg import PoseWithCovarianceStamped,  PoseStamped
from move_base_msgs.msg import  MoveBaseActionResult, MoveBaseActionGoal
from rospy.core import NullHandler  

class Goal_reached():  
    def __init__(self):  
        self.i =0
        rospy.init_node('goal_reached', anonymous=True)   
        self.goal_sub = rospy.Subscriber('/waypoint_mark', PoseWithCovarianceStamped, self.goal_callback1)
        self.result_sub = rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.result_callback)
        self.result_sub = rospy.Subscriber('/move_base/goal', MoveBaseActionGoal, self.goal_callback3)
        self.result_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback2)
        self.goal_pub = rospy.Publisher('/goal_back', PoseWithCovarianceStamped, queue_size=10)
        
    def result_callback(self,msg):
        if msg.status.status ==3 :
            print ("goal_reach")
            #self.i -= 1
            #print(self.i)
            #if self.i == 0 :
            print(self.x , self.y , self.z , self.w)
            self.goal = PoseWithCovarianceStamped()  
            self.goal.pose.pose.position.x = self.x
            self.goal.pose.pose.position.y = self.y
            self.goal.pose.pose.orientation.z = self.z
            self.goal.pose.pose.orientation.w = self.w
            self.goal_pub.publish(self.goal)
                
            
    def goal_callback1(self,msg):
        if msg != NullHandler :
            self.i+= 1
            self.x = msg.pose.pose.position.x
            self.y = msg.pose.pose.position.y
            self.z = msg.pose.pose.orientation.z
            self.w = msg.pose.pose.orientation.w
            #print(self.x , self.y , self.z , self.w)
            #print(self.i)
          
    def goal_callback2(self,Pose):
        if Pose != NullHandler :
            self.i+= 1
            self.x = Pose.pose.position.x
            self.y = Pose.pose.position.y
            self.z = Pose.pose.orientation.z
            self.w = Pose.pose.orientation.w
            #print(self.x , self.y , self.z , self.w)
            #print(self.i)    
    def goal_callback3(self,Goal):
         if Goal != NullHandler :
            self.x = Goal.target_pose.position.x
            self.y = Goal.target_pose.position.y
            self.z = Goal.target_pose.orientation.z
            self.w = Goal.target_pose.orientation.w
            print(self.x , self.y , self.z , self.w)      

if __name__ == '__main__':  
    try:  
        Goal_reached()  
        rospy.spin()  

    except rospy.ROSInterruptException:  
        rospy.loginfo("Exploring SLAM finished.")
