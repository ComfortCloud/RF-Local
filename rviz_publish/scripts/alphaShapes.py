#!/usr/bin/python
# -*- coding: UTF-8 -*-

import numpy as np
import math
import matplotlib.pyplot as plt
#import alphashape
import os
import sys
import pandas as pd
#from descartes import PolygonPatch
import matplotlib.pyplot as plt
sys.path.insert(0, os.path.dirname(os.getcwd()))
# import alphashape
from nav_msgs.msg import OccupancyGrid
import rospy


# 读取地图文件
def readMap():
    data = []
    file = open('/home/yunshu/taggroup_ws/src/rviz_publish/scripts/output','r')  #打开文件
    file_data = file.readlines() #读取所有行
    for row in file_data:
        row.strip('\n')
        tmp_list = row.split('\t') #按‘，’切分每行的数据
        tmp_list.pop()
        data.append(tmp_list) #将每行数据插入data中

    map = []
    for i in data:
        temp = []
        for j in i:
            temp.append(int(j))
        map.append(temp)

    print(type(map))

    return map


# 填写OccupancyGrid.msg
def grid_publisher():
	# ROS节点初始化
    rospy.init_node('gripMap_publisher', anonymous=True)

	# 创建一个Publisher
    info_pub = rospy.Publisher('/RFID_map', OccupancyGrid, queue_size=10)

	#设置循环的频率
    rate = rospy.Rate(10)

    # 创建map
    Map = readMap()
    height = len(Map)
    width = len(Map[0])

    # 初始化map_publisher::OccupancyGrid类型的消息
    myMap = OccupancyGrid()

    myMap.header.frame_id = "map"
    myMap.info.map_load_time = rospy.Time.now()
    myMap.info.resolution = 0.2
    myMap.info.width = 2* width
    myMap.info.height = 2* height
    myMap.info.origin.position.x = 0
    myMap.info.origin.position.y = 0
    myMap.info.origin.position.z = 0
    myMap.info.origin.orientation.x = 0
    myMap.info.origin.orientation.y = 0
    myMap.info.origin.orientation.z = 0
    myMap.info.origin.orientation.w = 0 

    data = []
    for i in range(myMap.info.height):
        temp = []
        for j in range(myMap.info.width):
            temp.append(-1)
        data.append(temp)

    data_inner = []
    for i in range(height):
        temp_inner = []
        for j in range(width):
            if Map[i][j] == 2:
                temp_inner.append(100)
            elif Map[i][j] == 1                                                                                                        :
                temp_inner.append(0)
            else:
                temp_inner.append(50)
        data_inner.append(temp_inner)

    start_line = math.ceil(height/2)
    start_col = math.ceil(width/2)

    data_mat = np.array(data)
    data_inner_mat = np.array(data_inner)

    print(type(data_mat),data_mat.shape,data_inner_mat.shape)

    data_mat[start_line : (start_line + height), start_col : (start_col + width)] = data_inner_mat

    mapData = []
    for i in range(myMap.info.height):
        for j in range(myMap.info.width):
            mapData.append(data_mat[i,j])
    myMap.data = mapData

    while not rospy.is_shutdown():
		# 发布消息
        info_pub.publish(myMap)
        rospy.loginfo("Publish message GridMap")

		# 按照循环频率延时
        rate.sleep()


if __name__ == '__main__':
    try:
        grid_publisher()
    except rospy.ROSInterruptException:
        pass
      


    # path = 'dataset.xlsx'

    # data = pd.read_excel(path)
    # x = list(data.x)
    # y = list(data.y)


    # # 存储数据
    # points_2d = []
    # points_list = []
    # for i in range(len(x)):
    #     med = (x[i],y[i])
    #     med_list = [x[i],y[i]]
    #     points_2d.append(med)
    #     points_list.append(med_list)


    # '''
    # 优先：从MATLAB 拷贝已经计算好的gridmap,Python只需负责生成OccupancyGrid.msg

    # 1. 周一给存储数据增加误差：按照3σ准则
    # 2. 移植MATLAB的货架匹配算法
    # 3. 将地图用rviz展示出来
    # '''

    # print(points_2d)

    # fig, ax = plt.subplots()
    # ax.scatter(*zip(*points_2d))
    # plt.show()

    # # alpha shapes 进行凹包检测
    # alpha_shape = alphashape.alphashape(points_2d, 0.1)
    # a = list(alpha_shape.exterior.coords)
    # print(list(alpha_shape.exterior.coords))
    # xt = []
    # yt = []
    
    # # 筛选出边界点
    # edge = []
    # for i in range(len(a)):
    #     xt.append(a[i][0])
    #     yt.append(a[i][1])
    #     edge.append([xt[i], yt[i]])

    # # 筛选出内部物品点
    # inner = []
    # flag_inner = 1
    # for i in range(len(points_list)):
    #     for j in range(len(edge)):
    #         if points_list[i][0] == edge[j][0]:
    #             if points_list[i][1] == edge[j][1]:
    #                 flag_inner = 0
    #     if flag_inner == 1:
    #         inner.append(points_list[i])
