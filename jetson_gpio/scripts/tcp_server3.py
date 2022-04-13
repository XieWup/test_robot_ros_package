#!/usr/bin/env python
# -*- coding: utf-8 -*-
from socket import *
from time import ctime
import time
import rospy 
import threading
from jetson_gpio.msg import data

def main():
  global cmd_pub,state
  state=True
  datas=0
  
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
      print(recv_1)
      print(recv_2)
      print(recv_3)
      print(recv_4)
      print(recv_5)
      print(recv_6)
      print(recv_7)
      print(recv_8)
      if state==True:
         msg=data()

         msg.data=datas
         cmd_pub.publish(msg)

      if recv_1==0 and datas==0 :
         print('控制继电器通道1常开吸合发送：AT+STACH1=1\\r\\n')
         meg11 = "AT+STACH1=1\r\n"
         tcpCliSock.send(meg11.encode())
         recv_data11 = tcpCliSock.recv(BUFSIZ)
         time.sleep(2)
         datas=1
         
      elif recv_1==0 and recv_5==0 and datas==1 :
         print('控制继电器通道1常开断开发送：AT+STACH1=1\\r\\n')
         meg9 = "AT+STACH1=0\r\n"
         meg10 = "AT+STACH5=1\r\n"
         tcpCliSock.send(meg9.encode())
         recv_data9 = tcpCliSock.recv(BUFSIZ)
      #   time.sleep(0.5)
         tcpCliSock.send(meg10.encode())
         
         recv_data10 = tcpCliSock.recv(BUFSIZ)
         time.sleep(2)
         datas=2
      elif recv_2==0 and recv_4==0 and datas==2 :
         meg12 = "AT+STACH2=1\r\n"
         meg13 = "AT+STACH5=0\r\n"
         tcpCliSock.send(meg12.encode())
         recv_data9 = tcpCliSock.recv(BUFSIZ)
      #   time.sleep(0.5)
         tcpCliSock.send(meg13.encode())
         
         recv_data10 = tcpCliSock.recv(BUFSIZ)
         datas=3
      
      
#def gpio_node():
    
  


if __name__ == '__main__':
    
    rospy.init_node('gpio')
    HOST = '192.168.1.105'
    PORT = 6000 #端口号
    BUFSIZ = 1024 #接收数据缓冲大小
    ADDR = (HOST, PORT)

    print('本机作为服务端')
    print('本机IP：')
    print(HOST)
    print('端口：')
    print(PORT)
    tcpSerSock = socket(AF_INET, SOCK_STREAM) #创建TCP服务器套接字
    tcpSerSock.bind(ADDR) #套接字与地址绑定
    tcpSerSock.listen(5) #监听连接，同时连接请求的最大数目


    print('等待客户机的连接')
    tcpCliSock, addr = tcpSerSock.accept()  #接收继电器端连接请求
    print('连接成功')
    print('客户端IP与端口如下:')
    print(addr)
  #  gpio_node()
    global cmd_pub
    cmd_pub = rospy.Publisher('tcp',data)
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


