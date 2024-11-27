# -*- coding: utf-8 -*-
import time
import socket
from datetime import datetime, timezone
import math
import threading
import csv
import matplotlib.pyplot as plt
import numpy as np
import rclpy
from rclpy.node import Node
from statistics import mean
from sensor_msgs.msg import TimeReference
from geometry_msgs.msg import Vector3Stamped  # geome

class IMUSubscribe(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info("大家好，我是%s!" % name)
        
        # 初始化订阅者
        self.euler_subscribe_node = self.create_subscription(Vector3Stamped, '/filter/euler', self.euler_callback, 100)  # QoS 参数
        self.positionlla_subscribe_node = self.create_subscription(Vector3Stamped, '/filter/positionlla', self.positionlla_callback, 100)
        self.velocity_callback_subscribe_node = self.create_subscription(Vector3Stamped, '/filter/velocity', self.velocity_callback, 100)
        self.utctime_callback_subscribe_node = self.create_subscription(TimeReference, '/imu/utctime', self.utctime_callback, 100)

        self.positionlla_values = []
        self.velocity_values = []
        self.longitude_values = []
        self.latitude_values = []
        self.heave1_values = []
        self.roll_values = []
        self.pitch_values = []
        self.yaw_values = []
        self.start_time = time.time()
        self.timestamp_utc = 0
        self.total_velocity = 0
        self.longitude = 0
        self.latitude = 0
        self.heave = 0
        self.heave1 = 0
        self.file = None
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.pi = 3.14159265358979323846
        self.Lx = 0.5 #天线距船中心X距离
        self.Ly = 5 #天线距船中心Y距离

       
        #plt.ion()
        #self.fig, self.axs = plt.subplots(6, 1, figsize=(10, 12))




        # 每十分钟创建新文件的定时器
        self.create_file() # 初始化第一个文件
        self.create_timer(600, self.create_file) # 每600秒（10分钟）执行一次


    def create_file(self):
        # 关闭当前文件（如果存在）
        if self.file:
            self.file.close()

        # 创建新文件
        self.file_name = time.strftime('%Y-%m-%d %H-%M-%S', time.localtime())
        self.file_path = '/home/jian/M680G/dataset'
        self.file = open(f'{self.file_path}/{self.file_name}.csv', 'w', newline='')  # 打开新文件以写入
        self.write_header()  # 写入文件头


    def write_header(self):
        # 写入CSV文件头
        header = ['timestamp', 'total_velocity', 'longitude', 'latitude', 'heave', 'roll', 'pitch', 'yaw']
        csv_writer = csv.writer(self.file)
        csv_writer.writerow(header)


    def write_data(self,total_velocity, longitude, latitude, heave, roll, pitch, yaw):
        timestamp = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime())
        # 获取当前时间
        # current_time = time.localtime()
        # # 将时间戳转换为字符串，精确到小数点后两位
        # timestamp = time.strftime('%Y-%m-%d %H:%M:%S', current_time) + f'.{current_time.tm_sec % 100:02d}'
        # 获取当前时间戳，精确到小数点后四位
        data = [timestamp, total_velocity, longitude, latitude, heave, roll, pitch, yaw]
        self.get_logger().info(f'时间戳 timestamp: {timestamp}')
        self.get_logger().info(f'数据名称: timestamp，total_velocity, longitude, latitude, heave, roll, pitch, yaw')  # 添加日志
        self.get_logger().info(f'写入数据: {data[1:]}')  # 添加日志'
        csv_writer = csv.writer(self.file)
        csv_writer.writerow(data)



    def utctime_callback(self, time_data):
        #self.get_logger().info(f'收到 time 数据:')
        # 获取 UTC 时间
        utc_time = datetime.utcfromtimestamp(time_data.time_ref.sec)
        # 转换为时间戳，转换为年月日时分秒格式，精确到小数点后两位
        timestamp_utc = utc_time.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]# 去掉最后的三个零
        

        # 打印时间戳
        # self.get_logger().info(f'  UTC时间: {time_data.time_ref.sec}')
        # self.get_logger().info(f'  时间戳: {timestamp_utc}')
        # self.get_logger().info(f'utctime回调函数被调用')
        
    def tcp_connect(self,ip, port):
            matrix = np.array([[self.total_velocity, self.longitude, self.latitude, self.heave, self.roll, self.pitch, self.yaw]])
        
            # TCP连接
            tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            tcp_socket.connect((ip, port))
        
            
            # 将矩阵转换为字节流
            data = matrix.tobytes()
            tcp_socket.send(data)
            print('----------发送数据------------')
            print(matrix)
            print('********************************')
            # 关闭 TCP 连接
            tcp_socket.close()

    def euler_callback(self, euler_data):
        
        self.roll, self.pitch, self.yaw = self.transform_euler(
            euler_data.vector.x,
            euler_data.vector.y,
            euler_data.vector.z,
            )
        # self.get_logger().info(f'  ROLL: {euler_data.vector.x}')
        # self.get_logger().info(f'  PITCH: {euler_data.vector.y}')
        # self.get_logger().info(f'  YAW: {euler_data.vector.z}')
        # self.get_logger().info(f'euler回调函数被调用')

        #记录数据
        #if self.total_velocity and self.longitude and self.latitude and self.heave:
        self.write_data(self.total_velocity, self.longitude, self.latitude, self.heave, self.roll, self.pitch, self.yaw)

        # 处理绘图数据
        
        if len(self.positionlla_values) > 250:
            self.velocity_values.append(self.total_velocity)
            self.longitude_values.append(self.longitude)
            self.latitude_values.append(self.latitude)
            self.heave1_values.append(self.heave1)
            self.roll_values.append(self.roll)
            self.pitch_values.append(self.pitch)
            self.yaw_values.append(self.yaw)


    def update_plot(self):
        if len(self.roll_values) > 0:
            t = np.arange(len(self.roll_values)) * 0.02

        # 清除旧图形
        for ax in self.axs:
            ax.clear()

        # 画图
        self.axs[0].plot(t, self.velocity_values)
        self.axs[0].set_title('Velocity')

        self.axs[1].plot(self.longitude_values, self.latitude_values)
        self.axs[1].set_title('Longitude vs Latitude')

        self.axs[2].plot(t, self.heave1_values)
        self.axs[2].set_title('Heave')

        self.axs[3].plot(t, self.roll_values)
        self.axs[3].set_title('Roll')

        self.axs[4].plot(t, self.pitch_values)
        self.axs[4].set_title('Pitch')

        self.axs[5].plot(t, self.yaw_values)
        self.axs[5].set_title('Yaw')

        # 刷新图形
        plt.draw()
        plt.pause(0.01) # 暂停以更新图形
        plt.show()


            
            
     # self.get_logger().info(f'positionlla回调函数被调用')
    # def imu_callback(self, imu_data):
    #     # 转换四元数为欧拉角
    #     roll, pitch, yaw = self.quaternion_to_euler(
    #         imu_data.orientation.x,
    #         imu_data.orientation.y,
    #         imu_data.orientation.z,
    #         imu_data.orientation.w
    #     )
    #     self.get_logger().info(f'  ROLL: {roll}, PITCH: {pitch}, YAW: {yaw}')

    #     # 记录数据
    #     if self.total_velocity and self.longitude and self.latitude and self.heave:
    #         self.write_data(self.total_velocity, self.longitude, self.latitude, self.heave, roll, pitch, yaw)
    #     self.get_logger().info(f'imu回调函数被调用')

    def positionlla_callback(self, positionlla_data):
        #self.get_logger().info(f'  经度: {positionlla_data.vector.y}')
        #self.get_logger().info(f'  纬度: {positionlla_data.vector.x}')
        delta_heave = self.heave_correct(self.pitch, self.roll, self.Lx,self.Ly)
        heave1 = positionlla_data.vector.z
        if len(self.positionlla_values) > 250:
                heave = positionlla_data.vector.z - mean(self.positionlla_values[:250]) - delta_heave #垂荡修正
               
        else:
                self.positionlla_values.append(positionlla_data.vector.z- delta_heave) #垂荡修正
                heave = positionlla_data.vector.z                 
                #self.get_logger().info(f'  垂荡: {heave}')

        
        
        #heave = positionlla_data.vector.z
        self.start_time = time.time()
        # self.heave = heave  # 存储垂荡值
        # self.longitude = positionlla_data.vector.y  # 存储经度
        # self.latitude = positionlla_data.vector.x  # 存储纬度
        self.heave1 = float(format(heave1, '.4f'))  
        self.heave = float(format(heave, '.4f'))  # 存储垂荡值，保留4位小数
        self.longitude = float(format(positionlla_data.vector.y, '.6f'))  # 存储经度，保留4位小数
        self.latitude = float(format(positionlla_data.vector.x, '.6f'))  # 存储纬度，保留4位小数

        #self.get_logger().info(f'  垂荡: {heave}')
        # self.get_logger().info(f'positionlla回调函数被调用')



    def velocity_callback(self, velocity_data):
        total_velocity = (velocity_data.vector.x**2 + velocity_data.vector.y**2 + velocity_data.vector.z**2)**0.5
        #self.total_velocity = total_velocity  # 存储总速度
        self.total_velocity = float(format(total_velocity, '.4f'))   # 存储总速度，保留4位小数
        # self.get_logger().info(f'  总速度: {total_velocity}')
        # self.get_logger().info(f'velocity回调函数被调用')

    def transform_euler(self, x, y, z):
       # 计算 Roll, Pitch, Yaw
        self.roll = float(format(x, '.4f'))
        self.pitch = float(format(y, '.4f'))
        self.yaw = float(format(z, '.4f'))

        return self.roll, self.pitch, self.yaw
    
    def heave_correct(self, pitch,roll,Lx,Ly):
         pi = 3.14159265358979323846
         delta_heave = Ly*math.sin(pitch*pi/180) - Lx*math.sin(roll*pi/180)
         return delta_heave

    def __del__(self):
        if self.file:
            self.file.close()  # 确保在节点销毁时关闭文件

def main(args=None):
    rclpy.init(args=args)  # 初始化 rclpy
    node = IMUSubscribe("imu_subscribe")  # 新建一个节点
    node.tcp_connect('192.168.31.49', 1000)  # 连接 TCP 服务器
    rclpy.spin(node)  # 保持节点运行
    rclpy.shutdown()  # 关闭 rclpy


if __name__ == '__main__':
    main()
    