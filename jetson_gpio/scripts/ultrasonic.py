#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from serial import Serial
import serial.tools.list_ports
import time
import struct
from std_msgs.msg import Float32
import math 
import os,sys

ultrasonic_pub =b'\xb5\x01'
byte={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}
base=[str(x) for x in range(10)]+[chr(x) for x in range(ord('A'),ord('A')+6)]
class TrdDriver():

    def __init__(self, serialprot, baudrate):
        
        self.sensor_pub = rospy.Publisher('sensor',Float32,queue_size=1)
           
        self.ser = serial.Serial(serialprot, baudrate)
        self.ser.write(ultrasonic_pub)
        print(self.ser)
        global sensor_pub
       # while True:

          
         # print(ultrasonic_pub)
       # while True: 
           #   self.ser = serial.Serial(serialprot, baudrate)
           #   self.ser.write(ultrasonic_pub)
        while self.ser.inWaiting()>0:
               read = self.ser.read(24)
               print(read)
               #byte =(struct.unpack("24b",read))
               #sun1=self.dec2hex(byte[2])
               #sun2=self.dec2hex(byte[3])
               #sun3=self.dec2hex(byte[8])
               #sun4=self.dec2hex(byte[9])
               #sun5=self.dec2hex(byte[14])
               #sun6=self.dec2hex(byte[15])
               #sun7=self.dec2hex(byte[20])
               #sun8=self.dec2hex(byte[21])   
               #date_1=float(sun1+sun2)
               #print(date_1)
               #date_2=int(sun3+sun4)
               #date_3=int(sun5+sun6)
               #date_4=int(sun7+sun8)
               #print(date_4)
               
               sensor_pub.publish(read)
               time.sleep(0.02)
          
    def dec2hex(self,numc):
        num=int(numc)
        mid=[]
       
        while True:
              if num==0:break
              num,rem=divmod(num,16)
              mid.append(base[rem])
              
        return ''.join([str(x) for x in mid[::-1]])


  


  
if __name__ =='__main__':
    
    #rospy.init_node('ultrasonic_node')   
    serialprot = rospy.get_param('~serialprot', default='/dev/ttyUSB0')
    baudrate = rospy.get_param('~baudrate', default=9600)
    
    TrdDriver(serialprot, baudrate)
    
    #rospy.spin()
