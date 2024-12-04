#!/usr/bin/env python3
# coding=utf-8

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from nlink_parser_ros2_interfaces.msg import LinktrackNodeframe2
import numpy as np

class Fusion(Node):
    def __init__(self):
        super().__init__('fusion')
        self.subscription1 = self.create_subscription(
            Point,
            'camera_topic',
            self.direct_callback,
            10
        )
        self.subscription1  # prevent unused variable warning

        self.subscription2 = self.create_subscription(
            LinktrackNodeframe2,  # 使用自定义消息类型
            '/nlink_linktrack_nodeframe2',
            self.fusion_callback,
            10  # 队列大小
        )
        self.subscription2  # 防止被垃圾回收

        # Publisher to publish fused data
        self.publisher = self.create_publisher(Point, 'fused_position', 10)

        self.tmp_x = 0
        self.tmp_z = 0

    def direct_callback(self, msg):
        self.tmp_x = msg.x
        self.tmp_z = msg.z
        # self.get_logger().info(f"Received message: x={self.tmp_x}, z={self.tmp_z}")

    def fusion_callback(self, msg):
        # self.get_logger().info(f"get UWB")
        if msg.nodes and len(msg.nodes) > 0:
                dis_value = msg.nodes[0].dis  # 假设我们要获取第一个节点的 dis 值

                self.get_logger().info(f'x={self.tmp_x * dis_value}, z={self.tmp_z * dis_value}, angle={np.degrees(np.arctan2(self.tmp_z, self.tmp_x))}')
                fused_position = Point()
                fused_position.x = self.tmp_x * dis_value
                fused_position.z = self.tmp_z * dis_value
                fused_position.y = 0.0  # If needed, you can set `y` to another value

                self.publisher.publish(fused_position)

def main(args=None):
    rclpy.init(args=args)
    node = Fusion()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
