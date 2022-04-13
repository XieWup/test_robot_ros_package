#!/usr/bin/env python
# -*- coding: utf-8 -*-
from socket import *
from time import ctime
import time
import rospy 
import threading
from jetson_gpio.msg import data
from std_msgs.msg import String

def main():
  global cmd_pub,location,m,switch
  location=0
  m=0
  switch = ''
  while True:
   #if state2==True :
     #while True:
     
      print('TCP-KP系列网络继电器功能测试：')
   # if state1==False and state2==True :
  #   print('控制继电器通道1常开吸合发送：AT+STACH1=1\\r\\n')
  #   meg11 = "AT+STACH1=1\r\n"
  #   tcpCliSock.send(meg11.encode())
   #  recv_data = tcpCliSock.recv(BUFSIZ)
  #   state1=True
      print('获取开关量输入接口的状态发送：AT+OCCH0=?\\r\\n')
      meg1 = "AT+OCCH1=?\r\n"
      meg2 = "AT+OCCH2=?\r\n"
      meg3 = "AT+OCCH3=?\r\n"
      meg4 = "AT+OCCH4=?\r\n"
      meg5 = "AT+OCCH5=?\r\n"
      meg6 = "AT+OCCH6=?\r\n"
      meg7 = "AT+OCCH7=?\r\n"
      meg8 = "AT+OCCH8=?\r\n"
      tcpCliSock.send(meg1.encode())
      recv_data1 = tcpCliSock.recv(BUFSIZ)
    #time.sleep(0.1)
      tcpCliSock.send(meg2.encode())
      recv_data2 = tcpCliSock.recv(BUFSIZ)
    #time.sleep(0.1)
      tcpCliSock.send(meg3.encode())
      recv_data3 = tcpCliSock.recv(BUFSIZ)
    #time.sleep(0.1)
      tcpCliSock.send(meg4.encode())
      recv_data4 = tcpCliSock.recv(BUFSIZ)
    #time.sleep(0.1)
      tcpCliSock.send(meg5.encode())
      recv_data5 = tcpCliSock.recv(BUFSIZ)
   # time.sleep(0.1)
      tcpCliSock.send(meg6.encode())
      recv_data6 = tcpCliSock.recv(BUFSIZ)
   # time.sleep(0.1)
      tcpCliSock.send(meg7.encode())
      recv_data7 = tcpCliSock.recv(BUFSIZ)
    #time.sleep(0.1)
      tcpCliSock.send(meg8.encode())
      recv_data8 = tcpCliSock.recv(BUFSIZ)
      print('网络继电器应答：')
      recv_data1=recv_data1.replace("\r\n", "\\r\\n").decode('gbk')
      recv_data2=recv_data2.replace("\r\n", "\\r\\n").decode('gbk')
      recv_data3=recv_data3.replace("\r\n", "\\r\\n").decode('gbk')
      recv_data4=recv_data4.replace("\r\n", "\\r\\n").decode('gbk')
      recv_data5=recv_data5.replace("\r\n", "\\r\\n").decode('gbk')
      recv_data6=recv_data6.replace("\r\n", "\\r\\n").decode('gbk')
      recv_data7=recv_data7.replace("\r\n", "\\r\\n").decode('gbk')
      recv_data8=recv_data8.replace("\r\n", "\\r\\n").decode('gbk')
  #recv_data1=recv_data[7]
  #recv_data2=recv_data[18]
  #recv_data3=recv_data[31]
    #recv_datas1=recv_data1.split(':')
    #recv_datas5=recv_data5.split(':')
      recv_1=int(recv_data1[7])
      recv_2=int(recv_data2[7])
      recv_3=int(recv_data3[7])
      recv_4=int(recv_data4[7])
      recv_5=int(recv_data5[7])
      recv_6=int(recv_data6[7])
      recv_7=int(recv_data7[7])
      recv_8=int(recv_data8[7])
      msg=data()

      msg.data1=recv_1
      msg.data2=recv_2
      msg.data3=recv_3
      msg.data4=recv_4
      msg.data5=recv_5
      msg.data6=recv_6
      msg.data7=recv_7
      msg.data8=recv_8

      cmd_pub.publish(msg)
      print(recv_1)
      print(recv_2)
      print(recv_3)
      print(recv_4)
      print(recv_5)
      print(recv_6)
      print(recv_7)
      print(recv_8)
      

      if location==1 and m==0 and switch=='map1':   #1-2
         print('控制继电器通道1常开吸合发送：AT+STACH1=1\\r\\n')
         meg11 = "AT+STACH1=1\r\n"
         tcpCliSock.send(meg11.encode())
         recv_data11 = tcpCliSock.recv(BUFSIZ)
         m=1
        
         
      elif recv_1==0 and recv_5==1 and location==1 and m==1 :
         print('控制继电器通道1常开断开发送：AT+STACH1=0\\r\\n')
         print('控制继电器通道5常开吸合发送：AT+STACH5=1\\r\\n')
         meg9 = "AT+STACH1=0\r\n"
         meg10 = "AT+STACH5=1\r\n"
         tcpCliSock.send(meg9.encode())
         recv_data9 = tcpCliSock.recv(BUFSIZ)
      #   time.sleep(0.5)
         tcpCliSock.send(meg10.encode())
         
         recv_data10 = tcpCliSock.recv(BUFSIZ)
         m=2

      elif location==2 and m==2 :

         print('控制继电器通道5常开断开发送：AT+STACH5=0\\r\\n')
         print('控制继电器通道7常开吸合发送：AT+STACH7=1\\r\\n')
         meg12 = "AT+STACH7=1\r\n"
         meg13 = "AT+STACH5=0\r\n"
         tcpCliSock.send(meg12.encode())
         recv_data9 = tcpCliSock.recv(BUFSIZ)
      #   time.sleep(0.5)
         tcpCliSock.send(meg13.encode())
         
         recv_data10 = tcpCliSock.recv(BUFSIZ)
         m==3

      elif recv_7==0 and recv_5==1 and location==2 and m==3 : 

         print('控制继电器通道7常开断开发送：AT+STACH7=0\\r\\n')
         print('控制继电器通道5常开吸合发送：AT+STACH5=1\\r\\n')
         meg14 = "AT+STACH7=0\r\n"
         meg15 = "AT+STACH5=1\r\n"
         tcpCliSock.send(meg14.encode())
         recv_data9 = tcpCliSock.recv(BUFSIZ)
      #   time.sleep(0.5)
         tcpCliSock.send(meg15.encode())
         
         recv_data10 = tcpCliSock.recv(BUFSIZ)
         m=4

      elif location==3 and m==4 :  
         print('控制继电器通道5常开断开发送：AT+STACH5=0\\r\\n')
         meg16 = "AT+STACH5=0\r\n"
         tcpCliSock.send(meg16.encode())
         recv_data9 = tcpCliSock.recv(BUFSIZ)
         m=5

      elif location==1 and m==5 and switch=='map2':    #2-3
         print('控制继电器通道2常开吸合发送：AT+STACH2=1\\r\\n')
         meg17 = "AT+STACH2=1\r\n"
         tcpCliSock.send(meg17.encode())
         recv_data11 = tcpCliSock.recv(BUFSIZ)
         m=6

      elif recv_2==0 and recv_5==1 and location==1 and m==6 :
         print('控制继电器通道2常开断开发送：AT+STACH2=0\\r\\n')
         print('控制继电器通道5常开吸合发送：AT+STACH5=1\\r\\n')
         meg18 = "AT+STACH2=0\r\n"
         meg19 = "AT+STACH5=1\r\n"
         tcpCliSock.send(meg18.encode())
         recv_data9 = tcpCliSock.recv(BUFSIZ)
      #   time.sleep(0.5)
         tcpCliSock.send(meg19.encode())
         
         recv_data10 = tcpCliSock.recv(BUFSIZ)
         m=7

      elif location==2 and m==7 :

         print('控制继电器通道5常开断开发送：AT+STACH5=0\\r\\n')
         print('控制继电器通道8常开吸合发送：AT+STACH8=1\\r\\n')
         meg20 = "AT+STACH8=1\r\n"
         meg21 = "AT+STACH5=0\r\n"
         tcpCliSock.send(meg20.encode())
         recv_data9 = tcpCliSock.recv(BUFSIZ)
      #   time.sleep(0.5)
         tcpCliSock.send(meg21.encode())
         
         recv_data10 = tcpCliSock.recv(BUFSIZ)
         m==8

      elif recv_8==0 and recv_5==1 and location==2 and m==8 : 

         print('控制继电器通道8常开断开发送：AT+STACH8=0\\r\\n')
         print('控制继电器通道5常开吸合发送：AT+STACH5=1\\r\\n')
         meg22 = "AT+STACH8=0\r\n"
         meg23 = "AT+STACH5=1\r\n"
         tcpCliSock.send(meg22.encode())
         recv_data9 = tcpCliSock.recv(BUFSIZ)
      #   time.sleep(0.5)
         tcpCliSock.send(meg23.encode())
         
         recv_data10 = tcpCliSock.recv(BUFSIZ)
         m=9

      elif location==3 and m==9 :  
         print('控制继电器通道5常开断开发送：AT+STACH5=0\\r\\n')
         meg24 = "AT+STACH5=0\r\n"
         tcpCliSock.send(meg24.encode())
         recv_data9 = tcpCliSock.recv(BUFSIZ)
         m=10

      elif location==1 and m==10 and switch=='map3' :    #3-1
         print('控制继电器通道4常开吸合发送：AT+STACH4=1\\r\\n')
         meg25 = "AT+STACH4=1\r\n"
         tcpCliSock.send(meg25.encode())
         recv_data11 = tcpCliSock.recv(BUFSIZ)
         m=11

      elif recv_4==0 and recv_5==1 and location==1 and m==11 :
         print('控制继电器通道4常开断开发送：AT+STACH4=0\\r\\n')
         print('控制继电器通道5常开吸合发送：AT+STACH5=1\\r\\n')
         meg26 = "AT+STACH4=0\r\n"
         meg27 = "AT+STACH5=1\r\n"
         tcpCliSock.send(meg26.encode())
         recv_data9 = tcpCliSock.recv(BUFSIZ)
      #   time.sleep(0.5)
         tcpCliSock.send(meg27.encode())
         
         recv_data10 = tcpCliSock.recv(BUFSIZ)
         m=12

      elif location==2 and m==12 :

         print('控制继电器通道5常开断开发送：AT+STACH5=0\\r\\n')
         print('控制继电器通道6常开吸合发送：AT+STACH6=1\\r\\n')
         meg28 = "AT+STACH6=1\r\n"
         meg29 = "AT+STACH5=0\r\n"
         tcpCliSock.send(meg28.encode())
         recv_data9 = tcpCliSock.recv(BUFSIZ)
      #   time.sleep(0.5)
         tcpCliSock.send(meg29.encode())
         
         recv_data10 = tcpCliSock.recv(BUFSIZ)
         m==13

      elif recv_6==0 and recv_5==1 and location==2 and m==13 : 

         print('控制继电器通道6常开断开发送：AT+STACH6=0\\r\\n')
         print('控制继电器通道5常开吸合发送：AT+STACH5=1\\r\\n')
         meg30 = "AT+STACH6=0\r\n"
         meg31 = "AT+STACH5=1\r\n"
         tcpCliSock.send(meg30.encode())
         recv_data9 = tcpCliSock.recv(BUFSIZ)
      #   time.sleep(0.5)
         tcpCliSock.send(meg31.encode())
         
         recv_data10 = tcpCliSock.recv(BUFSIZ)
         m=14

      elif location==3 and m==14 :  
         print('控制继电器通道5常开断开发送：AT+STACH5=0\\r\\n')
         meg32 = "AT+STACH5=0\r\n"
         tcpCliSock.send(meg32.encode())
         recv_data9 = tcpCliSock.recv(BUFSIZ)
         m=0
      
def gpio_node():
    rospy.Subscriber('location', data, vel_callback)
    rospy.Subscriber('/map_reload',String,callback1)


def vel_callback(msg):
    global location
    location=msg.data1
    

def vel_callback1(tring):
    global switch
    switch = tring  


if __name__ == '__main__':
    
    rospy.init_node('gpio')
   # HOST = '192.168.1.105'
   # PORT = 6000 #端口号
    BUFSIZ = 1024 #接收数据缓冲大小
  #  ADDR = (HOST, PORT)

  #  print('本机作为服务端')
  #  print('本机IP：')
  #  print(HOST)
  #  print('端口：')
  #  print(PORT)
    tcpCliSock = socket(AF_INET, SOCK_STREAM) #创建TCP服务器套接字
   # tcpSerSock.bind(ADDR) #套接字与地址绑定
   # tcpSerSock.listen(5) #监听连接，同时连接请求的最大数目
    server_ip="192.168.1.100"
    server_port=6000
    tcpCliSock.connect((server_ip,server_port))
 #   print('等待客户机的连接')
#    tcpCliSock, addr = tcpSerSock.accept()  #接收继电器端连接请求
##    print('连接成功')
 #   print('客户端IP与端口如下:')
#    print(addr)
  #  gpio_node()
    global cmd_pub
    cmd_pub = rospy.Publisher('IO',data)
    gpio_node()
    main()
    rospy.spin()
 #   print('关闭客户端连接！')
 #   tcpCliSock.close() #关闭与继电器的连接
 #   tcpSerSock.close() #关闭服务器socket
#print('测试完成！')
  #  rospy.spin()
    

#================================================
 # print('控制继电器通道1常开断开发送：AT+STACH1=0\\r\\n')
 # meg = "AT+STACH1=0\r\n"
 # tcpCliSock.send(meg.encode())
 # recv_data = tcpCliSock.recv(BUFSIZ)
 # print('网络继电器应答：')
 ## print(recv_data.decode('gbk'))
 # print('指令执行成功!')
 # time.sleep(0.5)#休眠0.5秒

#================================================
#print('控制继电器通道1常开接口吸合15秒后断开发送：AT+STACH1=1,15\\r\\n')
#meg = "AT+STACH1=1,15\r\n"
#tcpCliSock.send(meg.encode())
#recv_data = tcpCliSock.recv(BUFSIZ)
#print('网络继电器应答：')
#print(recv_data.decode('gbk'))
#print('指令执行成功!')
#time.sleep(0.5)#休眠0.5秒


  
#print('关闭客户端连接！')
#tcpCliSock.close() #关闭与继电器的连接
#tcpSerSock.close() #关闭服务器socket
#print('测试完成！')


