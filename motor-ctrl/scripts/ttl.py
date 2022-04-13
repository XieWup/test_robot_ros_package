#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from serial import Serial
import serial.tools.list_ports
import struct
import threading
import time


x=serial.Serial('/dev/ttyUSB1',9600)
def jieshou():
    while True:
      while x.inWaiting()>4:
         myout=x.read(4)
         #print(myout[3])
         checksum=myout[0]+myout[1]+myout[2]
         checksum_low=checksum&0xff
         if checksum_low==myout[3] :
            data=myout
            #print(data)
            datas=data[1]*256+data[2]
            print(datas)
        # datas=''.join(map(lambda x:('/x' if len(hex(x))>=4 else '/x0')+hex(x)[2:],data))
       #  new_datas=datas[2:].split('/x')

if __name__ == '__main__':
  rospy.init_node('ttl')
  t1=threading.Thread(target=jieshou)
  t1.start()
