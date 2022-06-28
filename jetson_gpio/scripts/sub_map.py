#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import copy
import rospy
from nav_msgs.msg import OccupancyGrid
import cv2
from collections import defaultdict
from matplotlib import pyplot as plt
from std_msgs.msg import String, Int32MultiArray, Float32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time

class Reducing(object):
    def __init__(self):
        rospy.init_node('map_reducing', anonymous=False)

        self.last_map = OccupancyGrid()
        self.map_adj_pub = rospy.Publisher('/map_reducing', OccupancyGrid, queue_size=10,
                                           latch=True)  # 定义发布剔除冗余数据后的地图数据
        self.image_pub = rospy.Publisher('/image_websocket', Int32MultiArray, queue_size=10)
        rospy.Subscriber("/map", OccupancyGrid, self.map_callback, queue_size=10)  # 订阅地图数据
        self.getList = []

    def map_callback(self, msg):
	print(msg)
        map_adj = OccupancyGrid()
        map_adj.header.frame_id = '/map'
        map_adj.header.stamp = rospy.Time.now()
        map_adj.info = msg.info
        original = msg.data
        #print(original)  #<type 'tuple'>
        print("原始地图信息：")
        print(len(original), map_adj.info.width * map_adj.info.height, map_adj.info.width, map_adj.info.height,
              msg.info.origin.position.x, msg.info.origin.position.y)
        #print(original)
        # 原始地图转为灰度图后获取最大外接矩形
        img_tmp = self.map2grayscale(original)
        #img_gray = np.array(img_tmp).reshape(map_adj.info.height, map_adj.info.width)
        #img_gray = img_gray.astype(np.uint8)  # 转换为灰度图的格式
        #cv2.imwrite('copy_empire.jpg', img_gray)  # 保存图片
        #img = cv2.imread('copy_empire.jpg')
        #img = cv2.flip(img,1)
        #cv2.imwrite('copy_empire1.jpg', img)  # 保存图片
        img_tmp.insert(0,map_adj.info.width)
        img_tmp.insert(1,map_adj.info.height)
        #img_tmp.insert(2,msg.info.origin.position.x) 
        #img_tmp.insert(3,msg.info.origin.position.y)
        #self.getList.append(img_tmp)
        #left_top = Float32MultiArray()
        left_top = Int32MultiArray(data=img_tmp)
        self.image_pub.publish(left_top)
        #print(left_top)
        #time.sleep(10)
        # print(type(img_gray), img_gray.shape, img_gray.dtype)
        

    @staticmethod
    def map2grayscale(data):
        # 对于只有黑白颜色的灰度图，为单通道，一个像素块对应矩阵中一个数字，数值为 0 到 255 ，其中 0 表示最暗（黑色）， 255 表示最亮（白色）
        # map data:    100 occupied    -1 unknown    0 free
        # costmap data:    100 occupied    -1 unknown    0 free    99 inflation layer
        data_new = copy.deepcopy(list(data))
        for idx, value in enumerate(data):
            if value == -1:  # 未知区域
                data_new[idx] = 20
            elif value == 100: #被占用
                data_new[idx] = 0
            elif value == 0:
                data_new[idx] = 255 #空闲
            else :
                data_new[idx] = 150 #其他
        return data_new


def point_of_index(map_info, i):
    # 计算地图栅格的索引值对应的世界坐标点
    y = map_info.origin.position.y + (i / map_info.width) * map_info.resolution
    x = map_info.origin.position.x + (i - (i / map_info.width) * map_info.width) * map_info.resolution
    return [x, y]


if __name__ == '__main__':
    try:
        Reducing()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo('Reducing /map redundant data terminated.')
