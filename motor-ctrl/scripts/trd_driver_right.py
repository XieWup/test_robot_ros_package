#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
import serial.tools.list_ports
import struct
from binascii import unhexlify
import binascii
import crcmod
import threading
from sensor_msgs.msg import BatteryState
import time

rel_speed = 0
speed_v = 0      #mms
speed_a = 0   #mrads 

motor_wheelbase = 530      #mm 轮距
motor_wheeldiameter = 150   #mm轮径
reduction_rate = 20# 减速比 
resolution = 10000#  
        #  ROS param  set

linear_coef = rospy.get_param('~linear_coef ', default =1000 ) 
angular_coef = rospy.get_param('~angular_coef', default = 1000)
maxspeed_v = rospy.get_param('~maxspeed_v', default = 1000)
maxspeed_a = rospy.get_param('~maxspeed_a', default = 1000)

motor_speed_mode1 = b'\x01\x2F\x60\x60\x00\x03\x00\x00\x00\x0D'
motor_speed_mode2 = b'\x02\x2F\x60\x60\x00\x03\x00\x00\x00\x0C'
motor_start1= b'\x01\x2B\x40\x60\x00\x0F\x00\x00\x00\x25'
motor_start2 = b'\x02\x2B\x40\x60\x00\x0F\x00\x00\x00\x24'
motor_stop1 = b'\x01\x2B\x40\x60\x00\x06\x00\x00\x00\x2E'
motor_stop2 = b'\x02\x2B\x40\x60\x00\x06\x00\x00\x00\x2D'
Drop_current = b'\x01\x44\x24\x2A\x34\x2A\x01\xF4\x01\xF4\xF7\x43' 
Rising_current = b'\x01\x44\x24\x2A\x34\x2A\x08\x34\x08\x34\xF2\xE3'

class TrdDriver():

    def __init__(self, serialprot, baudrate):
        #self.ser = serial.Serial(serialprot, baudrate)
        self.ser = serial.Serial(serialprot, baudrate,timeout=0.5)
        rospy.init_node('trd_driver_node')
        self.set_speed(speed_v, speed_a) 
        self.vel_sub = rospy.Subscriber('smoothcmd_vel', Twist, self.vel_callback)
        #self.vel_sub = rospy.Subscriber('cmd_vel1', Twist, self.vel_callback2)
        self.vel_sub = rospy.Subscriber('cmd_vel', Twist, self.vel_callback2)
        #self.vel_sub = rospy.Subscriber('battery',BatteryState,self.BatteryCallback)
        self.v2=0
        self.a2=0
       
        #self.ser.write(motor_start)
        #rospy.sleep(0.5)
        rospy.spin()
    def send(self, speed_code1,speed_code2):   
       
        #self.ser.write(motor_speed_mode1)
        #self.ser.write(speed_code1)    
        #self.ser.write(motor_start1)
        #time.sleep(0.03)
        self.ser.write(motor_speed_mode2)    
        self.ser.write(speed_code2)
        self.ser.write(motor_start2)
        
        print(motor_speed_mode1)
        print(motor_speed_mode2)
        print(motor_start1)
        print(motor_start2)
        print(speed_code1)
        print(speed_code2)

    def set_code(self,id,speed):
        self._data_bytes = bytes()
        self.id = id
        self.func = 0x23
        self.index = 0x60FF
        self.sub_index = 0
        self.data_value = speed
        #self.chks = 0
        _bytes_no_chks = struct.pack('<BBHBi', self.id, self.func, self.index, self.sub_index, self.data_value)
        _data_no_chks = struct.unpack('<BBBBBBBBB', _bytes_no_chks)
        _bytes_chks = struct.pack('<B', self.get_chks(_data_no_chks))
        self._data_bytes = _bytes_no_chks + _bytes_chks
        print(_data_no_chks, self.chks)
        print(self._data_bytes)
        return self._data_bytes
    def get_chks(self, data):
        """
        设置校验码.

        :param data: list/tuple,校验数据
        :returns: int,校验码
        :raises: no exception
        """
        _chks = sum(data)
        self.chks = (-_chks) & 0xFF
        return self.chks
    def set_speed(self, speed_v, speed_a):
	
        speed_left=int(speed_v+speed_a*motor_wheelbase/2000)
        speed_right=int(-speed_v+speed_a*motor_wheelbase/2000)
        
    
        speed_1=int(speed_left * 60 * 512 * resolution * reduction_rate / 1875 / 3.14 / motor_wheeldiameter)
        speed_2=int(speed_right * 60 * 512 * resolution * reduction_rate / 1875 / 3.14 / motor_wheeldiameter)
        #self.set_code(1,2,speed_1,speed_2)
        speed_code1=self.set_code(1,speed_1)
        speed_code2=self.set_code(2,speed_2)
        print(speed_1)
        print(speed_2)
        self.send(speed_code1,speed_code2)            
        

    def vel_callback(self, msg):
        print('test msg')
        print('test msg', msg.linear.x, msg.angular.z)
       # v1 =int (linear_coef*msg.linear.x )
       # a1 = int(angular_coef*msg.angular.z)
        v1 =int (1000*msg.linear.x )
        a1 = int(1000*msg.angular.z)
        print (v1,a1)
	
        speed_v = v1
        if speed_v> 0:
           if speed_v <= maxspeed_v:
              speed_v = speed_v
           else:
              speed_v = maxspeed_v
              speed_v = speed_v
        else:
           if speed_v >= (-maxspeed_v):
              speed_v = speed_v
           else:
              speed_v =  (-maxspeed_v)
              speed_v = speed_v
        speed_a = a1
        if speed_a> 0:
           if speed_a >= maxspeed_a:
              speed_a = speed_a
           else:
              speed_a = maxspeed_a
              speed_a = speed_a
        else:
           if speed_a >= (-maxspeed_a):
              speed_a = speed_a
           else:
              speed_a =  (-maxspeed_a)
        
              speed_a = speed_a
        #print(speed_v,speed_a)

        self.set_speed(speed_v, speed_a)  


    def vel_callback2(self, cmd):
        print(cmd)
        #speed_v = (linear_coef*cmd.linear.x )
        speed_v = (1000*cmd.linear.x )
        #speed_a = (angular_coef*cmd.angular.z)
        speed_a = (1000*cmd.angular.z)
        self.set_speed(speed_v, speed_a)

           
    def BatteryCallback(self, value):
        #print(value)
        battery = value.current
        if battery > 0:
            self.ser.write(Drop_current)
        else:
            self.ser.write(Rising_current)



if __name__ =='__main__':
    serialprot = rospy.get_param('~serialprot', default='/dev/ttyS5')
    baudrate = rospy.get_param('~baudrate', default=38400)
    #m = TrdDriver(serialprot, baudrate)
    m = TrdDriver('/dev/ttyS5', 38400)
