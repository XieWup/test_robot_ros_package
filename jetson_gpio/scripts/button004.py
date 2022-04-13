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
    self.staue=True
    self.stame=False
    self.current=0
    self.rsoc=0
    self.linear_x=0
    self.angular_z=0  
    self.stm=False
    
    self.sub3 = rospy.Subscriber('battery',BatteryState,self.callback3)
    self.sub4 = rospy.Subscriber("driver", Bool, self.callback4)
    #self.sub5 = rospy.Subscriber("stm", Bool, self.callback5)
    self.cmd_pub = rospy.Publisher('cmd_vel',Twist,queue_size=1)
    self.cmd_puc = rospy.Publisher('infrared',Bool,queue_size=1)

  def callback3(self,value) :
    print(value)
    self.current=value.current
    self.rsoc=value.percentage
    if self.staue==True :
       if self.current<0 and self.stm==False :
          self.linear_x=0
          self.angular_z=0.1
       elif self.current<0 and self.stm==True :  #对射传感器
          self.linear_x=-0.1
          self.angular_z=0
       else :
          self.linear_x=0
          self.linear_z=0 
       cmd=Twist()
       cmd.linear.x=self.linear_x
       cmd.angular.z=self.angular_z
       self.cmd_pub.publish(cmd)
    if self.btn_state1==True and self.rsoc>0.4 :
       self.staue=False

  def callback4(self,data):
    print(data)
    self.stm=data
    if self.stm==True:
       nav=True
    else:
       nav=False 
    self.cmd_puc.publish(nav) 
   
def main():
   obc = simple_class()
   rospy.init_node('simple_class', anonymous=True)
   try:
      rospy.spin()
   except KeyboardInterrupt:
      print("Shutting down")

if __name__ == '__main__':
    main()