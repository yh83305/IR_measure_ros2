#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from reflict_estimator import ReflictEstimator

def main(args=None):
    rclpy.init(args=args)
    node = ReflictEstimator()
    
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

