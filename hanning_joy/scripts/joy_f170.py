#!/usr/bin/python
#coding=utf-8
import time
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
#from can_msgs.msg import Frame


class Joy_f170:
    def __init__(self):

        self.__line_x = 0
        self.__angle_z = 0
        self.__pub_cmd_vel = None #:
        self.__pub_can_vel = None
        #self.__start = False
        rospy.init_node("joy")
        rospy.Subscriber("joy",Joy,self.joy_callback)
        #self.__pub_cmd_vel = rospy.Publisher("/joy/cmd_vel",Twist,queue_size=100)
        self.__pub_cmd_vel = rospy.Publisher("/cmd_vel",Twist,queue_size=100)
        rospy.spin()
        

    def joy_callback(self,data):
	while not rospy.is_shutdown():
		if data.buttons[0] == 1 and data.buttons[5] == 1:
			self.__line_x = data.axes[1]*0.3
			self.__angle_z = data.axes[0]*(0.3)
			cmd= Twist()
                	cmd.linear.x=self.__line_x
                	cmd.angular.z=self.__angle_z
                	self.__pub_cmd_vel.publish(cmd)
		elif data.buttons[2] == 1 and data.buttons[5] == 1:
			self.__line_x = data.axes[1]*0.8
			self.__angle_z = data.axes[0]*(0.8)
			cmd= Twist()
                	cmd.linear.x=self.__line_x
                	cmd.angular.z=self.__angle_z
                	self.__pub_cmd_vel.publish(cmd)
		else:
			self.__line_x = data.axes[1]*0.0
			self.__angle_z = data.axes[0]*0.0
			cmd= Twist()
                	cmd.linear.x=self.__line_x
                	cmd.angular.z=self.__angle_z
                	self.__pub_cmd_vel.publish(cmd)


if __name__=="__main__":
	Joy_f170()
