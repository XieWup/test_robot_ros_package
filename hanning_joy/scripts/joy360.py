#!/usr/bin/python
#coding=utf-8
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
        self.__start = False
        rospy.init_node("joy")
        rospy.Subscriber("joy",Joy,self.joy_callback)
        #self.__pub_cmd_vel = rospy.Publisher("/cmd_vel",Twist,queue_size=100)
        self.__pub_cmd_vel = rospy.Publisher("/joy/cmd_vel",Twist,queue_size=100)
        

    def joy_callback(self,data):
        if data.buttons[10] == 1:
            self.__start = not self.__start
 #           send_data.dlc = 8
 #           send_data.id = 2
 #           send_data.data = [0x00, 0xDA, 0x00, 0x10, 0x00, 0x00, 0x00, 0x1F]
 #           if not self.__start:
 #               send_data.data[7] = 0x0F
 #           self.__pub_can_vel.publish(send_data)
 #           send_data.id = 3
 #           self.__pub_can_vel.publish(send_data)

        
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
