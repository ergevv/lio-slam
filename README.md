# [lio-slam](https://github.com/ergevv/lio-slam.git)

#### 警告:本项目仍在开发中，故存在些许Bug，请勿用于实际使用，但是用于教程讲解原理足矣

视频代码讲解：[https://www.bilibili.com/video/BV1DhUkYaEnY/?vd_source=0163f7e67ca59d0c9187be5e667dda82](https://www.bilibili.com/video/BV1DhUkYaEnY/?vd_source=0163f7e67ca59d0c9187be5e667dda82)

本项目主要基于LIO(激光、IMU)讲解slam的相关技术，包括：

1、gazebo仿真激光和IMU：[https://github.com/linzs-online/robot_gazebo.git](https://github.com/linzs-online/robot_gazebo.git)

2、预积分的推导（与VINS-MONO的基于中值积分的预积分算法一致）：[https://blog.csdn.net/ergevv/article/details/143165323](https://blog.csdn.net/ergevv/article/details/143165323)

3、零偏更新：[https://blog.csdn.net/ergevv/article/details/143274896](https://blog.csdn.net/ergevv/article/details/143274896)

4、使用ceres进行图优化及添加信息矩阵：[https://blog.csdn.net/ergevv/article/details/143333425](https://blog.csdn.net/ergevv/article/details/143333425)

5、ndt匹配：《自动驾驶与机器人中的slam技术》

6、边缘化求取先验约束：[https://blog.csdn.net/ergevv/article/details/143365353](https://blog.csdn.net/ergevv/article/details/143365353)


#### 代码总体逻辑：
1、使用gazebo仿真获取IMU和LIDAR数据，并使用bag文件保存数据。
<br>
2、使用control_bag.py播放bag，支持用户通过键盘控制来播放、暂停、恢复、单步执行、指定发布固定数量的话题或重置ROS消息包中的数据，方便调试
<br>
3、IMU初始化：采用静止评估法。连续订阅IMU10s的数据，分别计算线加速度和角速度的平均值和方差的模，将当前线加速度的平均值认为是重力测量值g，将线加速度减去g后重新计算平均值和方差，如果加速度和角速度的方差的模均比较小，则认为当前是静止的，测量是比较可靠的。取当前的均值作为零偏，当前方差作为噪声的协方差
<br>
4、数据对齐：每次订阅到IMU和LIDAR数据，将第k帧的LIDAR数据与第k+1帧的LIDAR数据及两帧之间的IMU数据整理出来。
<br>
5、预积分：对第k帧与第k+1帧的LIDAR数据之间的IMU数据进行预积分，得到预积分观测量及观测量之间的协方差矩阵。
<br>
6、预测位资：使用第k帧点云的位资$pose_{k}$以及预积分观测量，得到k+1时刻的位姿$pose^1_{k+1}$，使用该预测位资作为点云匹配的初始位姿。使用ndt算法匹配第k帧与第k+1帧的点云，得到第二个k+1时刻的位姿$pose^2_{k+1}$
<br>
7、图优化：
第一次图优化：
（1）$pose_{k}$设为固定值，不优化
（2）IMU自身的约束：预积分观测量、$pose_{k}$与$pose^1_{k+1}$之差形成的约束，信息矩阵为预积分时的协方差矩阵的逆
（3）点云匹配的约束：使用$pose1$与$pose2$
（4）边缘化：使用第一次优化得到的雅可比矩阵和残差计算海森矩阵和残差梯度（$-J^Tr$）。边缘化掉k时刻的参数块，保留k+1时刻的，然后再重新求解雅可比矩阵和残差
第二次图优化：
（1）边缘化：使用第一次图优化里的雅可比矩阵和残差，用来作为先验值，约束这次的k时刻的位资
（2）IMU自身的约束：预积分观测量、$pose_{k}$与$pose^1_{k+1}$之差形成的约束，信息矩阵为预积分时的协方差矩阵的逆
（3）点云匹配的约束：使用$pose1$与$pose2$
（4）更新边缘化
<br>
8、更新关键帧：判断当前帧与上一帧关键帧运动是否足够大，运动变化较大则认为是关键帧，合并这一帧到关键帧地图，并用来更新ndt的目标点云。


### 编译环境：

- PCL 1.14
- Ceres 1.14
- Eigen
- YAML
- Ubuntu 20.04 + ROS noetic


### 使用教程：

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
（2）可使用python脚本控制bag的播放，允许用户通过键盘控制来播放、暂停、恢复、单步执行或重置ROS消息包中的数据。[https://github.com/ergevv/Bag-publication-controller-in-ROS.git](https://github.com/ergevv/Bag-publication-controller-in-ROS.git)



<br>
<br>
<br>
<br>



#### WARNING: This project is still under development, so there are some bugs. Please do not use it for actual use, but it is sufficient for tutorial explanation of principles
This project is mainly based on LIO (laser, IMU) to explain the relevant technologies of SLAM, including:
1. Gazebo simulation laser and IMU:[https://github.com/linzs-online/robot_gazebo.git]
2. The derivation of pre integration (consistent with VINS-MONO's median integration based pre integration algorithm):[https://blog.csdn.net/ergevv/article/details/143165323]
3. Zero bias update:[https://blog.csdn.net/ergevv/article/details/143274896]
4. Using Ceres for graph optimization and adding information matrix:[https://blog.csdn.net/ergevv/article/details/143333425]
5. NDT Matching: SLAM Technology in Autonomous Driving and Robotics
6. To obtain prior constraints for marginalization:[https://blog.csdn.net/ergevv/article/details/143365353]


#### General Logic of the Code:
1. Use Gazebo simulation to obtain IMU and LIDAR data, and save the data using bag files.
2. Play back the bag using `control_bag.py`, supporting users to control playback, pause, resume, single-step execution, publish a fixed number of topics, or reset the data in the ROS message bag via keyboard for easier debugging.
3. IMU Initialization: Employ the static evaluation method. Continuously subscribe to 10 seconds of IMU data, calculate the mean and variance modulus of linear acceleration and angular velocity separately. Assume the current mean value of linear acceleration as the gravitational measurement \( g \), subtract \( g \) from the linear acceleration, and recalculate the mean and variance. If both the variance modulus of acceleration and angular velocity are relatively small, it is considered that the current state is static and the measurement is reliable. Take the current mean as the zero bias and the current variance as the covariance of noise.
4. Data Alignment: Each time IMU and LIDAR data are subscribed, organize the LIDAR data of the k-th frame with the LIDAR data of the (k+1)-th frame and the IMU data between these two frames.
5. Pre-integration: Perform pre-integration on the IMU data between the k-th and (k+1)-th frames of LIDAR data to obtain pre-integrated measurements and the covariance matrix between the measurements.
6. Pose Prediction: Use the pose \( pose_k \) of the point cloud at the k-th frame and the pre-integrated measurements to get the pose \( pose^1_{k+1} \) at time k+1, which serves as the initial pose for point cloud matching. Use the NDT algorithm to match the point clouds at the k-th and (k+1)-th frames to obtain the second pose \( pose^2_{k+1} \) at time k+1.
7. Graph Optimization:
First Graph Optimization:
(1) Set \( pose_k \) as a fixed value, not optimized.
(2) IMU's own constraints: Constraints formed by the difference between the pre-integrated measurements, \( pose_k \), and \( pose^1_{k+1} \), with the information matrix being the inverse of the covariance matrix during pre-integration.
(3) Point Cloud Matching Constraints: Use \( pose1 \) and \( pose2 \).
(4) Marginalization: Use the Jacobian matrix and residuals obtained from the first optimization to compute the Hessian matrix and residual gradient (\(-J^T r\)). Marginalize out the parameter block at time k, retain the one at time k+1, and then solve the Jacobian matrix and residuals again.
Second Graph Optimization:
(1) Marginalization: Use the Jacobian matrix and residuals from the first graph optimization as prior values to constrain the pose at time k in this optimization.
(2) IMU's own constraints: Constraints formed by the difference between the pre-integrated measurements, \( pose_k \), and \( pose^1_{k+1} \), with the information matrix being the inverse of the covariance matrix during pre-integration.
(3) Point Cloud Matching Constraints: Use \( pose1 \) and \( pose2 \).
(4) Update Marginalization.
8. Keyframe Update: Determine if the movement of the current frame relative to the last keyframe is sufficiently large; if the motion change is significant, consider it a keyframe, merge this frame into the keyframe map, and use it to update the target point cloud of the NDT.

### Compilation Environment:

PCL 1.14
Ceres 1.14
Eigen
YAML
Ubuntu 20.04 + ROS Noetic

### Usage Guide:

### Compile Catkin Workspace:
```sh
catkin_make to compile.
```
### Start and Operate Gazebo Simulation Environment:
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

### Perform SLAM Process:
(1) Start SLAM: 
```sh
roslaunch slam test.launch
```
(2) A Python script can be used to control the playback of ROS bag files, allowing users to control the playback, pause, resume, step through, or reset the data in the bag file via keyboard input. The relevant script can be obtained from the GitHub repository: [https://github.com/ergevv/Bag-publication-controller-in-ROS.git](https://github.com/ergevv/Bag-publication-controller-in-ROS.git)