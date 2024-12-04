#!/usr/bin/env python3
# coding=utf-8

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# 定义全局变量来存储数据
x_data, y_data, z_data = [], [], []

class ImageListener(Node):
    def __init__(self):
        super().__init__('image_listener')

        # 创建 3D 图形对象
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')

        # 绘制球面
        self.u = np.linspace(0, 2 * np.pi, 100)
        self.v = np.linspace(0, np.pi, 100)
        self.x_sphere = np.outer(np.cos(self.u), np.sin(self.v))
        self.y_sphere = np.outer(np.sin(self.u), np.sin(self.v))
        self.z_sphere = np.outer(np.ones(np.size(self.u)), np.cos(self.v))
        self.ax.plot_surface(self.x_sphere, self.y_sphere, self.z_sphere, color='b', alpha=0.1, edgecolor='w')

        # 创建订阅者
        self.subscription = self.create_subscription(
            Point,
            'camera_topic',
            self.direct_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        plt.show(block=False)

    def direct_callback(self, msg):
        # 更新数据
        x_data.append(msg.x)
        y_data.append(msg.y)
        z_data.append(msg.z)
        self.update_plot()

    def update_plot(self):
        self.ax.cla()  # Clear the current axis
        # 绘制球面
        self.ax.plot_surface(self.x_sphere, self.y_sphere, self.z_sphere, color='b', alpha=0.1, edgecolor='w')
        # 绘制散点图
        self.ax.scatter(x_data, y_data, z_data, c='r', marker='o')
        plt.draw()
        plt.pause(0.1)  # Pause to allow for the plot to update

def main(args=None):
    rclpy.init(args=args)
    node = ImageListener()
    rclpy.spin(node)
    plt.close(node.fig)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
