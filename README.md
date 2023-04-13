# 多传感器融合RFID定位
**操作系统： Ubunutu 18.04**



## RFID标签定位

### 数据采集

#### 实验场景介绍

天线安装位置：

![image-20220808114002385](C:\Users\tuin\AppData\Roaming\Typora\typora-user-images\image-20220808114002385.png)

标签位置：每一侧两列之间距离为30cm，两行之间距离为30cm

![image-20220808054114775](C:\Users\tuin\AppData\Roaming\Typora\typora-user-images\image-20220808054114775.png)



两边天线分别距各侧标签的垂直距离为30cm~40cm

读写器读取时间设置在30s，根据行驶距离调整

机器人移动速度在0.5m/s左右

注意：

- 移动速度必须要设置得尽量慢，才能使数据点光滑（0.5m/s是一个比较合适的选择）

- 场景内不要出现其他不希望测到的标签，会占用信道，减少目标标签的数据点
- 数据采集的好坏决定着定位精度

#### 前提工作：配置连接tracer机器人和读写器

- tracer机器人：

```
rosrun tracer_bringup bringup_can2usb.bash
```

- 读写器：


添加有线网络IPv4 115.156.142.225

测试：`ping 115.156.142.223`

#### 采集

- 启动机器人底盘：

```
roslaunch tracer_bringup tracer_robot_base.launch
```

- 若使用键盘控制：

```
roslaunch tracer_bringup tracer_teleop_keyboard.launch
```

若使用导航来采集数据，详情见导航部分操作

- 开启订阅读写器数据和机器人里程计节点

```
roslaunch package_launch localdata.launch
```

注意：需要在两个订阅节点reader_node.cpp和odom_publish.cpp文件中修改成你自己的路径

### 数据处理

测试：在Matlab中运行process.m可以看到解缠相位-时间曲线是否合理，以此判断数据是否可以使用

计算：运行C++程序Project/src/main.cpp

在文件中对相关参数进行了注释，使用时注意修改数据文件路径。

运行后即可在终端输出标签坐标

这里标签坐标信息保存在EPCxyz文件中，可在rviz中展现出来（注意修改EPC.publish中EPCxyz的路径为你自己的路径）：

```
roscore
cd catkin_ws
rosrun rviz_publish EPC_publish
```

发布了EPC形状和EPC标签信息

在Fixed Frame将map改为EPC，Add MakerArray类型改即可在rviz中查看



## SLAM & Navigation 功能包

### 依赖项
- OpenCV 3.x
- `sudo apt-get ros-melodic-navigation`
- `sudo apt install ros-melodic-teleop-twist-keyboard`
- `sudo apt install ros-melodic-joint-state-publisher-gui`
- `sudo apt install ros-melodic-ros-controllers`
- `sudo apt install ros-melodic-webots-ros`
- `sudo apt install libasio-dev`
### Cartographer
Google推出的开源2D & 3D SLAM功能包，在任务中用作2D建图。

**Cartographer文件夹为一个单独的工作空间，因为编译方法有区别于传统的工作空间，在Cartographer文件夹中的README文件有说明编译方法。**


### SLAM建图过程
首先先启动机器人底盘以及激光雷达的节点：
```
roslaunch velodyne_pointcloud VLP16_points.launch
rosrun tracer_bringup bringup_can2usb.bash
roslaunch tracer_bringup tracer_robot_base.launch
```
其次启动机器人底盘的键盘控制节点，手动控制机器人的移动：
```
roslaunch tracer_bringup tracer_teleop_keyboard.launch
```
手动控制机器人的过程也可以通过开启导航并随机发布目标点来实现，此时不加载地图，默认全局为无限平坦的地区，当雷达扫描到障碍物时也可以进行自动避障，但是该功能并没有实现，因为该建图过程较为浪费时间。

然后开启Cartographer导航并录制点云数据（**其中由于Cartographer是另外的功能包因此需要在终端中进行额外的source操作**）：

```
source ~/cartographer/devel_isolate/setup.bash
roslaunch cartographer_ros VLP16_2d.launch
rosbag record -o out /velodyne_points
```
在cartographer建图完成后需要保存地图并关闭rosbag的录制：
```
rosrun map_server map_saver -f /your_path
```
回放点云数据并进行三维建图：
```
roslaunch lego_loam run.launch
rosbag play out_xxx.bag --clock --topic /velodyne_points
```
### Navigation导航过程
首先先启动机器人底盘以及激光雷达的节点：
```
roslaunch velodyne_pointcloud VLP16_points.launch
rosrun tracer_bringup bringup_can2usb.bash
roslaunch tracer_bringup tracer_robot_base.launch
```
将之前的地图文件`.pgm`与`.yaml`复制到`/mbot_navigation/maps`文件夹中，并修改`/mbot_navigation/launch/nav_cloister_demo.launch`中的`<arg name="map" default="map11.yaml" />`部分。

可以在`/mbot_navigation/config/mbot/base_local_planner_params.yaml`中的`max_vel_x`与`min_vel_x`修改导航速度，但是不能太慢，可能会导致机器人不断的转圈前进。

其次开启导航节点：

```
roslaunch mbot_navigation nav_cloister_demo.launch
```
最后发布目标点信息即可实现导航，在这里我发布了两个目标点，并在第二个点进行了90度的转向，到达第一个目标点后会有3s的停顿：
```
rosrun mbot_navigation destination_publish
```