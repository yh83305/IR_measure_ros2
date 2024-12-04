import rclpy 
from rclpy.node import Node
from sensor_msgs_py import point_cloud2
from sensor_msgs.msg import PointCloud2, LaserScan
from std_msgs.msg import Float32MultiArray,Bool
import math
import numpy as np
from geometry_msgs.msg import Point

class ReflictEstimator(Node):
    def __init__(self):
        super().__init__("reflict_estimator")
        self.__sub_ouster = self.create_subscription(
            PointCloud2, "/ouster/points", self.ouster_cb, 10)
        self.__reflict_pub = self.create_publisher(
            Point, '/reflict_center', 10)
        self._reflict_point_pub = self.create_publisher(
            PointCloud2, "reflict/points", 10)
        pass

    def euler_to_quaternion(self, yaw):
        """Convert yaw angle to quaternion."""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)

        q = {
            'x': 0.0,
            'y': 0.0,
            'z': sy,
            'w': cy
        }

        return q

    def ouster_cb(self, data):
        gen = np.array(list(point_cloud2.read_points(data, skip_nans=True)))
        data_raw = gen[:, :4]
        x = data_raw[:, 0]
        y = data_raw[:, 1]
        z = data_raw[:, 2]
        intensity = data_raw[:, 3]
        theta = np.deg2rad(30)
        dir_pos = np.where(x > abs(y)*np.tan(theta))
        x = x[dir_pos]
        y = y[dir_pos]
        z = z[dir_pos]
        intensity = intensity[dir_pos]
        # print(max(intensity))
        points = np.stack([x, y, z, intensity], axis=1)
        fanguang = np.where(intensity > 3000)
        points = points[fanguang]

        points_2d = np.stack([x, y], axis=1)
        points_2d = points_2d[fanguang]
        distance = np.linalg.norm(points_2d, axis=1)
        dis_constrain = np.where(distance < 10)
        points_2d = points_2d[dis_constrain]
        # print(points_2d)/home/gxh/Desktop/bags/read.py

        if points_2d.shape[0] > 0:
            point_cloud = point_cloud2.create_cloud(
                data.header, data.fields[0:4], points)
            self._reflict_point_pub.publish(point_cloud)
            center = np.mean(points_2d, axis=0)
            print("center:", -center[1], center[0])
            center_point = Point()
            center_point.x = -center[1]
            center_point.y = 0.0
            center_point.z = center[0]
            self.__reflict_pub.publish(center_point)
        pass