#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import sys
from std_msgs.msg import Bool
import actionlib  
from actionlib_msgs.msg import *  
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist  
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import BatteryState
from jetson_gpio.msg import data
from std_msgs.msg import String

class simple_class:

  def __init__(self):
    
    self.sub = rospy.Subscriber('location', data, self.callback)
    self.sub1 = rospy.Subscriber('IO', data, self.callback1)
 
    #self.cmd_pub = rospy.Publisher('mapswitch',data,queue_size=10)
    self.cmd_pub1 = rospy.Publisher('stype',data,queue_size=10)
    self.pub = rospy.Publisher('/map_reload', String, queue_size=10)
    self.switch = 'map1'

     # 订阅move_base服务器的消息  
    self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)  
    rospy.loginfo("Waiting for move_base action server...")  

    while self.move_base.wait_for_server(rospy.Duration(5.0)) == 0:
        rospy.loginfo("Connected to move base server") 

  def callback(self,data):   
    self.state = data.data1
    if self.state==1 and self.recv_1==0 and self.recv_5==1 :
       self.target = Pose(Point(0.000, 0.001, 0.000), Quaternion(0.000, 0.000, 0.013,0.999))
    #充电点
       self.goal = MoveBaseGoal()  
       self.goal.target_pose.pose = self.target
       self.goal.target_pose.header.frame_id = 'map'  
       self.goal.target_pose.header.stamp = rospy.Time.now() 
       rospy.loginfo("Going to: " + str(self.target)) 
    # 向目标进发  
       self.move_base.send_goal(self.goal)  

    # 五分钟时间限制  
       self.finished_within_time = self.move_base.wait_for_result(rospy.Duration(300))   

    # 查看是否成功到达  
       if not self.finished_within_time:  
           self.move_base.cancel_goal()  
           rospy.loginfo("Timed out achieving goal")  
       else:  
           state = self.move_base.get_state()  
           if state == GoalStatus.SUCCEEDED:  
               rospy.loginfo("Goal succeeded!")
               staue=1
            #print("self.staue is :" + self.staue)
           else:  
               rospy.loginfo("Goal failed！ ")
    #self.staue=data.data
    if staue==1 :

      if  self.recv_5==0 and recv_6==1 :
          self.switch ='map1'
      if  self.recv_5==0 and recv_7==1 :
          self.switch ='map2'
      if  self.recv_5==0 and recv_8==1 :
          self.switch ='map3'

    self.pub.publish(self.switch)
    rospy.loginfo(self.switch)

    if staue==1 and self.recv_5==1 and (self.recv_6==0 or self.recv_7==0 or self.recv_8==0) :
       self.target = Pose(Point(0.000, 0.001, 0.000), Quaternion(0.000, 0.000, 0.013,0.999))
    #充电点
       self.goal = MoveBaseGoal()  
       self.goal.target_pose.pose = self.target
       self.goal.target_pose.header.frame_id = 'map'  
       self.goal.target_pose.header.stamp = rospy.Time.now() 
       rospy.loginfo("Going to: " + str(self.target)) 
    # 向目标进发  
       self.move_base.send_goal(self.goal)  

    # 五分钟时间限制  
       self.finished_within_time = self.move_base.wait_for_result(rospy.Duration(300))   

    # 查看是否成功到达  
       if not self.finished_within_time:  
           self.move_base.cancel_goal()  
           rospy.loginfo("Timed out achieving goal")  
       else:  
           state = self.move_base.get_state()  
           if state == GoalStatus.SUCCEEDED:  
               rospy.loginfo("Goal succeeded!")
               staue=2

            #print("self.staue is :" + self.staue)
           else:  
               rospy.loginfo("Goal failed！ ")
    cmd=data()
    cmd.data1= staue
    self.cmd_pub1.publish(cmd)    



  def callback1(self,data):
        self.recv_1=data.data1
        self.recv_2=data.data2
        self.recv_3=data.data3
        self.recv_4=data.data4
        self.recv_5=data.data5
        self.recv_6=data.data6
        self.recv_7=data.data7
        self.recv_8=data.data8
    

def main():
  
  try:
    rospy.init_node('simple_class', anonymous=True)
    obc = simple_class()
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main()
