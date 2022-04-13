#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import rospy 
import Jetson.GPIO as GPIO
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion    
import time
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
import threading

# Pin Definitons:
red_pin_out = 33  # Board pin 12
green_right_out = 36
green_left_out = 37
blue_pin_out = 35

scram_pin_in = 7 #scram in

anti_collision_in = 15
infrared_in = 18
charge_out =38
scram_pin_out = 40

def callback1( value):
    global current,rsoc
    current= value.current
    rsoc= value.percentage
    #print(current)
def callback2(nav):
    global nac
    nac=nav.data
    #print(nac)

def vel_callback(msg):
    global linear,angular
    linear=msg.linear.x 
    angular=msg.angular.z
    #print(linear, angular)

def main():
    global current,rsoc,linear, angular,cmd_pub,nac
    current=0
    rsoc=0
    linear=0
    angular=0
    nac=False
    cmd_pub = rospy.Publisher('driver',Bool,queue_size=1)
    # Pin Setup:
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)  # BOARD pin-numbering scheme
    GPIO.setup(red_pin_out, GPIO.OUT)  # LED pin set as output
    GPIO.setup(green_right_out, GPIO.OUT)
    GPIO.setup(green_left_out, GPIO.OUT)
    GPIO.setup(blue_pin_out, GPIO.OUT)
    GPIO.setup(charge_out, GPIO.OUT)
    GPIO.setup(scram_pin_out, GPIO.OUT)


    GPIO.setup(scram_pin_in, GPIO.IN)  # button pin set as input
    GPIO.setup(anti_collision_in, GPIO.IN)
    GPIO.setup(infrared_in, GPIO.IN)
    curr_value = GPIO.LOW
    # Initial state for LEDs:
    GPIO.output(red_pin_out, GPIO.LOW)
    GPIO.output(green_right_out, GPIO.LOW)
    GPIO.output(green_left_out, GPIO.LOW)
    GPIO.output(blue_pin_out, GPIO.LOW)
    GPIO.setup(charge_out, GPIO.LOW)
    GPIO.output(scram_pin_out, GPIO.LOW)
    
    
    #print("Starting demo now! Press CTRL+C to exit")
   
    while True:
        
        if GPIO.input(scram_pin_in) == GPIO.LOW:
            GPIO.output(scram_pin_out, GPIO.LOW)
            time.sleep(1)
            GPIO.output(red_pin_out, curr_value)
            curr_value ^= GPIO.HIGH
        elif GPIO.input(anti_collision_in) == GPIO.HIGH and GPIO.input(scram_pin_in) == GPIO.HIGH :
    #       while True:
            GPIO.output(red_pin_out, GPIO.HIGH)
            GPIO.output(scram_pin_out, GPIO.LOW) 
        else:
            GPIO.output(red_pin_out, GPIO.LOW)
            GPIO.output(scram_pin_out, GPIO.HIGH)
        if current>0 and GPIO.input(scram_pin_in) == GPIO.HIGH and GPIO.input(anti_collision_in) == GPIO.LOW :
            
            time.sleep(1)
            GPIO.output(blue_pin_out, curr_value)
            curr_value ^= GPIO.HIGH
        elif linear==0 and angular==0 and rsoc>0.3 and current<0 and GPIO.input(scram_pin_in) == GPIO.HIGH and GPIO.input(anti_collision_in) == GPIO.LOW :
            GPIO.output(blue_pin_out, GPIO.HIGH)
            
        else:
            
            GPIO.output(blue_pin_out, GPIO.LOW)

        if rsoc<=0.3 and current<0 and GPIO.input(scram_pin_in) == GPIO.HIGH and GPIO.input(anti_collision_in) == GPIO.LOW :
            time.sleep(1)
            GPIO.output(green_right_out, curr_value)
            GPIO.output(green_left_out, curr_value)
            curr_value ^= GPIO.HIGH
        elif linear !=0 and angular == 0 and rsoc>0.3 and current<0 and GPIO.input(scram_pin_in) == GPIO.HIGH and GPIO.input(anti_collision_in) == GPIO.LOW :
             GPIO.output(green_right_out, GPIO.HIGH)
             GPIO.output(green_left_out, GPIO.HIGH)
        elif angular <0 and rsoc>0.3 and current<0 and GPIO.input(scram_pin_in) == GPIO.HIGH and GPIO.input(anti_collision_in) == GPIO.LOW : 
             GPIO.output(green_right_out, GPIO.HIGH)
             GPIO.output(green_left_out, GPIO.LOW)
        elif angular >0 and rsoc>0.3 and current<0 and GPIO.input(scram_pin_in) == GPIO.HIGH and GPIO.input(anti_collision_in) == GPIO.LOW : 
             GPIO.output(green_right_out,GPIO.LOW )
             GPIO.output(green_left_out, GPIO.HIGH)
        else:
            GPIO.output(green_right_out, GPIO.LOW)
            GPIO.output(green_left_out, GPIO.LOW)
        if nac==True:
           GPIO.output(charge_out, GPIO.HIGH)
        else:
           GPIO.output(charge_out, GPIO.LOW)

        if GPIO.input(infrared_in) == GPIO.LOW :
            
            data=True
        else:
          
            data=False
        cmd_pub.publish(data)

def gpio_node():
    rospy.Subscriber('battery',BatteryState,callback1)
    rospy.Subscriber('/smooth/cmd_vel', Twist, vel_callback) 
    rospy.Subscriber("infrared", Bool, callback2) 
   

if __name__ == '__main__':
    
    rospy.init_node('gpio')
    gpio_node()
    main()
    rospy.spin()
    
   
    
   
