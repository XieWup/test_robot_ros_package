#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import Bool
import actionlib  
from actionlib_msgs.msg import *  
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist  
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class buttons:

  def __init__(self):
    self.btn_state = False
    self.sub = rospy.Subscriber("btn_press", Bool, self.callback)
     # 订阅move_base服务器的消息  
    self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)  
    rospy.loginfo("Waiting for move_base action server...")  

    while self.move_base.wait_for_server(rospy.Duration(5.0)) == 0:
        rospy.loginfo("Connected to move base server") 

  def callback(self,data):
    self.target = Pose(Point(0.758, -2.957, 0.000), Quaternion(0.000, 0.000, -0.679, 0.734))
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
    self.btn_state = data.data


def main():
  
  try:
    
    rospy.init_node('button', anonymous=True)
    obc = buttons()
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main()
