#!/usr/bin/python
#coding=utf-8
from os import statvfs
import time
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
#from can_msgs.msg import Frame


class Joy360:
    def __init__(self):
        self.__line_x = 0
        self.__angle_z = 0
        self.__pub_cmd_vel = None #:
        self.__pub_can_vel = None
        self.__pub_sav= None
        self.__start = False
        rospy.init_node("joy")
        rospy.Subscriber("joy",Joy,self.joy_callback)
        #self.__pub_cmd_vel = rospy.Publisher("/cmd_vel",Twist,queue_size=100)
        self.__pub_cmd_vel = rospy.Publisher("/joy/cmd_vel",Twist,queue_size=100)
        self.__pub_sav = rospy.Publisher("waypoints_joy",Joy,queue_size=100)

    def joy_callback(self,data):
        if data.buttons[10] == 1:
            self.__start = not self.__start
            if data.buttons[3]==1:
                sav= 1
                self.__pub_sav.publish(sav)
        self.__line_x = data.axes[1]*0.3
        self.__angle_z = data.axes[3]*0.3
        
    def start(self):
        #print("start init this node")
       
        rospy.loginfo("init node ok!!")
             
#        self.__pub_can_vel = rospy.Publisher("/sent_messages",Frame,queue_size=100)
        try:
            while not rospy.is_shutdown():
                #time.sleep(0.01)
                cmd= Twist()
                cmd.linear.x=self.__line_x
                cmd.angular.z=self.__angle_z
                if self.__start:
                   
                   self.__pub_cmd_vel.publish(cmd)

        except KeyboardInterrupt as e:
             rospy.loginfo("quit by user " + str(e))


if __name__=="__main__":
    joy_ins = Joy360()
    joy_ins.start()