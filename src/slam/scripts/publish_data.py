import rospy
import rosbag
from std_msgs.msg import String
from sensor_msgs.msg import Imu
import numpy as np
from scipy.integrate import cumtrapz
import matplotlib.pyplot as plt



def main2():
    rospy.init_node('bag_player', anonymous=True)
    bag = rosbag.Bag('/home/erge/桌面/开源工程/ros_ws/src/slam/bag/pc_imu2.bag', 'r')
    
    i=0
    for topic, msg, t in bag.read_messages():
        # 发布消息
        pub = rospy.Publisher(topic, type(msg), queue_size=10)
        pub.publish(msg)
        i+=1
        if(i>=3000):
            input("Press Enter to publish the next message...")

    bag.close()




def read_imu_data(bag_file):
    # 初始化位置和时间列表
    times = []
    accelerations_x = []
    accelerations_y = []

    # 读取 bag 文件中的 IMU 数据
    with rosbag.Bag(bag_file, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=['/imu/data']):
            times.append(t.to_sec())
            accelerations_x.append(msg.linear_acceleration.x)
            accelerations_y.append(msg.linear_acceleration.y)

    return np.array(times), np.array(accelerations_x), np.array(accelerations_y)

def predict_position(times, accelerations_x, accelerations_y):
    # 计算速度变化
    velocities_x = cumtrapz(accelerations_x, x=times, initial=0)
    velocities_y = cumtrapz(accelerations_y, x=times, initial=0)

    # 计算位置变化
    positions_x = cumtrapz(velocities_x, x=times, initial=0)
    positions_y = cumtrapz(velocities_y, x=times, initial=0)

    return positions_x, positions_y

def plot_positions(times, positions_x, positions_y):
    plt.figure()

    # 绘制位置变化曲线
    plt.plot(positions_y, positions_x)
    plt.xlabel('Position Y (m)')
    plt.ylabel('Position X (m)')
    plt.show()

def main():
    bag_file = '/home/erge/桌面/开源工程/ros_ws/pc_imu.bag'  # 替换为你的 bag 文件路径
    times, accelerations_x, accelerations_y = read_imu_data(bag_file)
    positions_x, positions_y = predict_position(times, accelerations_x, accelerations_y)

    # 绘制位置变化曲线
    plot_positions(times, positions_x, positions_y)

if __name__ == '__main__':
    main2()

