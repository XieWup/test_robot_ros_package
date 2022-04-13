#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy 
import Jetson.GPIO as GPIO
from geometry_msgs.msg import Point32
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion    
import time
from std_msgs.msg import Int16
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
import threading

# Pin Definitons:
red_pin_out = 33  # Board pin 12
green_pin_out = 31
blue_pin_out = 32

scram_pin_in = 7 #scram in

fall_arrest_in = 19
anti_collision_in = 15
pause_pin_in = 16
pause_pin_out = 35
infrared_in = 18
infrared1_in = 11
infrared2_in = 12
infrared3_in = 13
plate_lamp1_out =36
plate_lamp2_out =37
plate_lamp3_out =38
scram_pin_out = 40

def callback1(msg):
    global current,rsoc
    current=msg.x
    rsoc=msg.y
    #print(current)


def vel_callback(msg):
    global linear,angular
    
    linear=msg.linear.x 
    angular=msg.angular.z
    #print(linear)

def main():
    # Pin Setup:
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)  # BOARD pin-numbering scheme
    GPIO.setup(red_pin_out, GPIO.OUT)  # LED pin set as output
    GPIO.setup(green_pin_out, GPIO.OUT)
    GPIO.setup(blue_pin_out, GPIO.OUT)
    GPIO.setup(pause_pin_out, GPIO.OUT)
    GPIO.setup(plate_lamp1_out, GPIO.OUT)
    GPIO.setup(plate_lamp2_out, GPIO.OUT)
    GPIO.setup(plate_lamp3_out, GPIO.OUT)
    GPIO.setup(scram_pin_out, GPIO.OUT)


    GPIO.setup(scram_pin_in, GPIO.IN)  # button pin set as input
    GPIO.setup(fall_arrest_in, GPIO.IN)
    GPIO.setup(anti_collision_in, GPIO.IN)
    GPIO.setup(pause_pin_in, GPIO.IN)
    GPIO.setup(infrared_in, GPIO.IN)
    GPIO.setup(infrared1_in, GPIO.IN)
    GPIO.setup(infrared2_in, GPIO.IN)
    GPIO.setup(infrared3_in, GPIO.IN)

    curr_value = GPIO.LOW
    # Initial state for LEDs:
    GPIO.output(red_pin_out, GPIO.LOW)
    GPIO.output(green_pin_out, GPIO.LOW)
    GPIO.output(blue_pin_out, GPIO.LOW)
    GPIO.output(pause_pin_out, GPIO.LOW)
    GPIO.output(plate_lamp1_out, GPIO.LOW)
    GPIO.output(plate_lamp2_out, GPIO.LOW)
    GPIO.output(plate_lamp3_out, GPIO.LOW)
    GPIO.output(scram_pin_out, GPIO.HIGH)
    global current,rsoc,data
    global linear, angular
    current=0
    rsoc=0
    linear=0
    angular=0
    
    print("Starting demo now! Press CTRL+C to exit")
   
    while True:
        
        if GPIO.input(scram_pin_in) == GPIO.LOW:
            GPIO.output(scram_pin_out, GPIO.LOW)
            time.sleep(1)
            GPIO.output(red_pin_out, curr_value)
            curr_value ^= GPIO.HIGH
        elif GPIO.input(fall_arrest_in) == GPIO.HIGH or GPIO.input(anti_collision_in) == GPIO.HIGH and GPIO.input(scram_pin_in) == GPIO.HIGH :
    #       while True:
            GPIO.output(red_pin_out, GPIO.HIGH) 
            GPIO.output(scram_pin_out, GPIO.LOW)
        else:
            GPIO.output(red_pin_out, GPIO.LOW)
            GPIO.output(scram_pin_out, GPIO.HIGH)
        if current>0 and GPIO.input(scram_pin_in) == GPIO.LOW and GPIO.input(fall_arrest_in) == GPIO.LOW and GPIO.input(anti_collision_in) == GPIO.LOW :
            
            time.sleep(1)
            GPIO.output(blue_pin_out, curr_value)
            curr_value ^= GPIO.HIGH
        elif linear==0 and angular==0 and rsoc>0.3 and current<0 and GPIO.input(scram_pin_in) == GPIO.LOW and GPIO.input(fall_arrest_in) == GPIO.LOW and GPIO.input(anti_collision_in) == GPIO.LOW :
            GPIO.output(blue_pin_out, GPIO.HIGH)
            
        else:
            
            GPIO.output(blue_pin_out, curr_value)

        if rsoc<=0.3 and current<0 and GPIO.input(scram_pin_in) == GPIO.LOW and GPIO.input(fall_arrest_in) == GPIO.LOW and GPIO.input(anti_collision_in) == GPIO.LOW :
            time.sleep(1)
            GPIO.output(green_pin_out, curr_value)
            curr_value ^= GPIO.HIGH
        elif linear !=0 or angular !=0 and rsoc>0.3 and current<0 and GPIO.input(scram_pin_in) == GPIO.LOW and GPIO.input(fall_arrest_in) == GPIO.LOW and GPIO.input(anti_collision_in) == GPIO.LOW :
             GPIO.output(green_pin_out, GPIO.HIGH)
        else:
            GPIO.output(green_pin_out, GPIO.LOW)

        if GPIO.input(pause_pin_in) == GPIO.HIGH :
            GPIO.output(pause_pin_out, GPIO.HIGH)
             
        else:
            GPIO.output(pause_pin_out, GPIO.LOW)
            

        if GPIO.input(infrared1_in) == GPIO.HIGH :
            GPIO.output(plate_lamp1_out,GPIO.HIGH)
        else:
            GPIO.output(plate_lamp1_out, GPIO.LOW)


        if GPIO.input(infrared2_in) == GPIO.HIGH :
            GPIO.output(plate_lamp2_out,GPIO.HIGH)
        else:
            GPIO.output(plate_lamp2_out, GPIO.LOW)


        if GPIO.input(infrared3_in) == GPIO.HIGH :
            GPIO.output(plate_lamp3_out,GPIO.HIGH)
        else:
            GPIO.output(plate_lamp3_out, GPIO.LOW)
        
        if current>0 or GPIO.input(pause_pin_in) == GPIO.HIGH or GPIO.input(fall_arrest_in) == GPIO.HIGH or GPIO.input(anti_collision_in) == GPIO.HIGH:
           data=0
        else:
           data=1
        if GPIO.input(infrared1_in) == GPIO.LOW and GPIO.input(infrared2_in) == GPIO.LOW and GPIO.input(infrared3_in) == GPIO.LOW :
           staue=True
        else:
           staue=False
        if GPIO.input(infrared_in) == GPIO.LOW :
           stase=True
        else:
           stase=False
           
        global cmd_pub, cmd_pubm, cmd_pubn
        cmd_pub.publish(data)
        cmd_pubm.publish(staue)
        cmd_pubn.publish(stase)
def gpio_node():
    global cmd_pub,cmd_pubm,cmd_pubn
    cmd_pub = rospy.Publisher('driver',Int16,queue_size=1)
    cmd_pubm = rospy.Publisher('staue',Bool,queue_size=1)
    cmd_pubn = rospy.Publisher('stase',Bool,queue_size=1)
    rospy.Subscriber('battery',Point32,callback1)
    rospy.Subscriber('cmd_vel', Twist, vel_callback)
    rospy.spin()

if __name__ == '__main__':
    
    rospy.init_node('gpio')
    t1=threading.Thread(target=main)
    t2=threading.Thread(target=gpio_node)
    t2.start()
    t1.start()