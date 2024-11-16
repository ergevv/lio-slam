import rosbag
import rospy
import threading
import time
from pynput import keyboard
import genpy.message

class BagPlayer:
    def __init__(self, bag_file):
        self.bag_file = bag_file
        self.bag = rosbag.Bag(self.bag_file)
        self.topics = self.bag.get_type_and_topic_info()[1].keys()
        self.publishers = {topic: rospy.Publisher(topic, self.get_message_class(topic), queue_size=10) for topic in self.topics}
        self.messages = list(self.bag.read_messages())
        self.current_index = 0
        self.running = False
        self.lock = threading.Lock()

    def get_message_class(self, topic):
        topic_types, topic_names = self.bag.get_type_and_topic_info()
        topic_type = topic_names[topic].msg_type
        return genpy.message.get_message_class(topic_type)

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self.run)
        self.thread.start()

    def stop(self):
        self.running = False
        self.thread.join()

    def run(self):
        while self.running and self.current_index < len(self.messages):
            with self.lock:
                if not self.running:
                    break
                topic, msg, t = self.messages[self.current_index]

                
                self.publishers[topic].publish(msg)
                self.current_index += 1
            time.sleep(0.01)  # 模拟发布间隔

    def pause(self):
        with self.lock:
            self.running = False

    def resume(self):
        with self.lock:
            self.running = True
            self.thread = threading.Thread(target=self.run)
            self.thread.start()

    def step(self):
        with self.lock:
            if self.current_index < len(self.messages):
                topic, msg, t = self.messages[self.current_index]
                self.publishers[topic].publish(msg)
                self.current_index += 1

    def reset(self):
        with self.lock:
            self.current_index = 0
            self.stop()

def get_number_input(prompt="请输入一个数字: "):
    while True:
        try:
            # 使用 input 函数获取用户输入
            user_input = input(prompt)
            # 尝试将输入转换为浮点数
            number = int(user_input)
            # 如果转换成功，则返回数字
            return number
        except ValueError:
            # 如果转换失败，说明输入不是有效的数字
            print("无效输入！请确保您输入的是一个数字。")


def main():
    rospy.init_node('bag_player', anonymous=True)
    bag_file = '/home/erge/桌面/开源工程/ros_ws/src/slam/bag/pc_imu2.bag'  # 替换为你的 bag 文件路径
    player = BagPlayer(bag_file)

    def on_press(key):
        try:
            key_char = key.char
            if key_char == 'p':  # 暂停
                player.pause()
            elif key_char == 'r':  # 恢复
                player.resume()
            elif key_char == 's':  # 单步播放
                player.step()
            elif key_char == 'q':  # 重头播放
                player.reset()
            elif key_char == 'b':  # 开始播放
                player.start()
            elif key_char == 'e':  # 停止播放
                player.stop()
            elif key_char == 'n':  # 播放次数
                num = get_number_input()
                for i in range(num):
                    player.step()
        except AttributeError:
            pass  # 特殊键（如 Ctrl、Shift 等）不会触发

    # 监听键盘事件
    listener = keyboard.Listener(on_press=on_press)
    listener.start()

    # 保持程序运行
    rospy.spin()

if __name__ == '__main__':
    main()