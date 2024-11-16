# lio-slam

# 警告:本项目仍在开发中，故存在些许Bug，请勿用于实际使用，但是用于教程讲解原理足矣

本项目主要基于LIO(激光、IMU)讲解slam的相关技术，包括：
1、gazebo仿真激光和IMU：[https://github.com/linzs-online/robot_gazebo.git]
1、预积分的推导（与VINS-MONO的基于中值积分的预积分算法一致）：[https://blog.csdn.net/ergevv/article/details/143165323]
2、零偏更新：[https://blog.csdn.net/ergevv/article/details/143274896]
3、使用ceres进行图优化及添加信息矩阵：[https://blog.csdn.net/ergevv/article/details/143333425]
4、ndt匹配：《自动驾驶与机器人中的slam技术》
5、边缘化求取先验约束：[https://blog.csdn.net/ergevv/article/details/143365353]


# 编译环境：

- PCL 1.14
- Ceres 1.14
- Eigen
- YAML
- Ubuntu 20.04 + ROS noetic


# 使用教程：

1、编译
```sh
catkin_make
```

2、
（1）启动gazebo仿真环境：
```sh
roslaunch scout_gazebo scout_gazebo.launch
```
（2）控制仿真里的机器人：
```sh
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```
（3）录制仿真里的激光、IMU数据：
```sh
rosbag record -O pc_imu  /velodyne_points  /imu/data
```

3、
（1）启动slam：
```sh
roslaunch slam test.launch
```
（2）可使用python脚本控制bag的播放，允许用户通过键盘控制来播放、暂停、恢复、单步执行或重置ROS消息包中的数据。[git@github.com:ergevv/Bag-publication-controller-in-ROS.git]







# WARNING: This project is still under development, so there are some bugs. Please do not use it for actual use, but it is sufficient for tutorial explanation of principles
This project is mainly based on LIO (laser, IMU) to explain the relevant technologies of SLAM, including:
1. Gazebo simulation laser and IMU:[https://github.com/linzs-online/robot_gazebo.git]
2. The derivation of pre integration (consistent with VINS-MONO's median integration based pre integration algorithm):[https://blog.csdn.net/ergevv/article/details/143165323]
3. Zero bias update:[https://blog.csdn.net/ergevv/article/details/143274896]
4. Using Ceres for graph optimization and adding information matrix:[https://blog.csdn.net/ergevv/article/details/143333425]
5. NDT Matching: SLAM Technology in Autonomous Driving and Robotics
6. To obtain prior constraints for marginalization:[https://blog.csdn.net/ergevv/article/details/143365353]




# Compilation Environment:

PCL 1.14
Ceres 1.14
Eigen
YAML
Ubuntu 20.04 + ROS Noetic

# Usage Guide:

# Compile Catkin Workspace:
```sh
catkin_make to compile.
```
# Start and Operate Gazebo Simulation Environment:
(1) Launch the Gazebo simulation environment: 
```sh
roslaunch scout_gazebo scout_gazebo.launch
```
(2) Control the robot in the simulation:
```sh
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```
(3) Record LiDAR and IMU data from the simulation: 
```sh
rosbag record -O pc_imu /velodyne_points /imu/data
```

# Perform SLAM Process:
(1) Start SLAM: 
```sh
roslaunch slam test.launch
```
(2) A Python script can be used to control the playback of ROS bag files, allowing users to control the playback, pause, resume, step through, or reset the data in the bag file via keyboard input. The relevant script can be obtained from the GitHub repository: [git@github.com:ergevv/Bag-publication-controller-in-ROS.git]