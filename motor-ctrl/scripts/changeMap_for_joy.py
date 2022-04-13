#!/usr/bin/python3
#coding=utf-8
import time
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class Joy360:
    def __init__(self):
        self.__start = False
        pass

    def joy_callback(self,data:Joy):
        if data.buttons[1] == 1:
            #self.__start = not self.__start
            while not rospy.is_shutdown():
                time.sleep(0.05)
                hello_str = 'map2'
                # 发布消息
                self.pub.publish(hello_str)
                rospy.loginfo(hello_str)
                break
        elif data.buttons[2] == 1:
                #self.__start = not self.__start
            while not rospy.is_shutdown():
                time.sleep(0.05)
                hello_str = 'map1'
                # 发布消息
                self.pub.publish(hello_str)
                rospy.loginfo(hello_str)
                break

    def start(self):
        #print("start init this node")
        rospy.init_node("joy_change", disable_signals=True)
        rospy.loginfo("init node ok!!")
        rospy.Subscriber("/joy",Joy,self.joy_callback)
        self.pub = rospy.Publisher('/map_reload', String, queue_size=10)
#        self.__pub_cmd_vel = rospy.Publisher("/turtle1/cmd_vel",Twist,queue_size=100)
#        self.__pub_can_vel = rospy.Publisher("/sent_messages",Frame,queue_size=100)
        rospy.spin()


if __name__=="__main__":
    joy_ins = Joy360()
    joy_ins.start()
