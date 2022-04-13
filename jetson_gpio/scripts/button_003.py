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
class simple_class:

  def __init__(self):
    self.btn_state = False
    self.btn_state1=False
    self.btn_state2=False
    self.staue=False
    self.stame=False
    self.current=0
    self.rsoc=0
    self.linear_x=0
    self.angular_z=0  
    self.stm=False
    self.stme=False
    self.nav=False
    self.sub = rospy.Subscriber("btn_press", Bool, self.callback)
    self.sub1 = rospy.Subscriber("btn_press1", Bool, self.callback1)
    self.sub2 = rospy.Subscriber("btn_press2", Bool, self.callback2)
    self.sub3 = rospy.Subscriber('battery',BatteryState,self.callback3)
    self.sub4 = rospy.Subscriber("driver", Bool, self.callback4)
    #self.sub5 = rospy.Subscriber("stm", Bool, self.callback5)
    self.cmd_pub = rospy.Publisher('cmd_vel1',Twist,queue_size=1)
    self.cmd_puc = rospy.Publisher('infrared',Bool,queue_size=1)

     # 订阅move_base服务器的消息  
    self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)  
    rospy.loginfo("Waiting for move_base action server...")  

    while self.move_base.wait_for_server(rospy.Duration(5.0)) == 0:
        rospy.loginfo("Connected to move base server") 

  def callback(self,data):   
    self.btn_state = True
    self.btn_state1=False
    self.btn_state2=False
    self.target = Pose(Point(-0.126, 0.049, 0.000), Quaternion(0.000, 0.000, 0.028, 0.999))
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
            self.staue=True
            print("self.staue is :" + self.staue)
        else:  
            rospy.loginfo("Goal failed！ ")
    #self.staue=data.data
   

  def callback1(self,data):
    self.btn_state = False
    self.btn_state1= True
    self.btn_state2=False      
    self.target = Pose(Point(0.993, -1.623, 0.000), Quaternion(0.000, 0.000, -0.701, 0.713))
    #取餐点
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
        else:  
            rospy.loginfo("Goal failed！ ")
 #   self.staue=data.data

  def callback2(self,data):
    self.btn_state = False
    self.btn_state1=False
    self.btn_state2=True
    self.target = Pose(Point(0.235, -3.877, 0.000), Quaternion(0.000, 0.000, -0.997, 0.077))
    #送餐点
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
            
        else:  
            rospy.loginfo("Goal failed！ ") 
    #self.staue=data.data


  def callback3(self,value) :
    print(value)
    self.current=value.current
    self.rsoc=value.percentage
    
    if self.rsoc<=0.2 and self.current<0 and self.staue==False and  self.btn_state == False  :
      self.target = Pose(Point(0.993, --1.623, 0.000), Quaternion(0.000, 0.000, -0.701, 0.713))
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
              self.staue=True
          else:  
              rospy.loginfo("Goal failed！ ") 


    if self.staue==True :
       
       if self.current<0 and self.stm==False and self.stme==False :
          self.linear_x=0
          self.angular_z=0.1
       elif self.current<0 and self.stm==False and self.stme==True :
          self.linear_x=0
          self.angular_z=-0.1
       elif self.current<0 and self.stm==True :  #对射传感器
          self.linear_x=-0.04
          self.angular_z=0
          self.stme=False
       else  :
          self.linear_x=0
          self.angular_z=0 
       cmd=Twist()
       cmd.linear.x=self.linear_x
       cmd.angular.z=self.angular_z
       self.cmd_pub.publish(cmd)
       if self.btn_state1==True or self.btn_state2==True and self.rsoc>0.4  :
          self.staue=False
          self.stme=False
          self.nav=False
      
  def callback4(self,data):
    print(data.data)
    self.stm=data.data
    if self.staue==True and self.stme==False and self.stm==False:
       rospy.sleep(5)
       self.stme=True
    if self.stm==True and self.staue==True:
       self.nav=True     
    self.cmd_puc.publish(self.nav) 

    

def main():
  
  try:
    rospy.init_node('simple_class', anonymous=True)
    obc = simple_class()
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main()
