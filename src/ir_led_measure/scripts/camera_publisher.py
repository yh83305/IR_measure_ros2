#!/usr/bin/env python3
# coding=utf-8

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np

import os

import mvsdk
from dscamera import DSCamera

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')

        self.publisher_ = self.create_publisher(Point, 'camera_topic', 10)
        self.bridge = CvBridge()
        
        self.cam = DSCamera("/home/gxh/Desktop/ir_ws/src/ir_led_measure/calibration.json", img_size = (1280, 1024), fov = 185)
        self.get_logger().info(f'FOV: {self.cam.fov}')
        self.get_logger().info(f'img_size: {self.cam.img_size}')
        self.get_logger().info(f'intrinsic: {self.cam.intrinsic}')
        self.get_logger().info("Start")

        # 打开相机
        self.DevList = mvsdk.CameraEnumerateDevice()
        self.nDev = len(self.DevList)
        if self.nDev < 1:
            self.get_logger().error("No camera was found!")
            return

        for i, DevInfo in enumerate(self.DevList):
            self.get_logger().info(f"{i}: {DevInfo.GetFriendlyName()} {DevInfo.GetPortType()}")
        i = 0 if self.nDev == 1 else int(input("Select camera: "))
        self.DevInfo = self.DevList[i]
        self.get_logger().info(str(self.DevInfo))

        # 打开相机
        self.hCamera = 0
        try:
            self.hCamera = mvsdk.CameraInit(self.DevInfo, -1, -1)
        except mvsdk.CameraException as e:
            self.get_logger().error(f"CameraInit Failed({e.error_code}): {e.message}")
            return

        # 获取相机特性描述
        self.cap = mvsdk.CameraGetCapability(self.hCamera)

        # 判断是黑白相机还是彩色相机
        self.monoCamera = (self.cap.sIspCapacity.bMonoSensor != 0)

        # 黑白相机让ISP直接输出MONO数据，而不是扩展成R=G=B的24位灰度
        if self.monoCamera:
            mvsdk.CameraSetIspOutFormat(self.hCamera, mvsdk.CAMERA_MEDIA_TYPE_MONO8)
        else:
            mvsdk.CameraSetIspOutFormat(self.hCamera, mvsdk.CAMERA_MEDIA_TYPE_BGR8)

        # 相机模式切换成连续采集
        mvsdk.CameraSetTriggerMode(self.hCamera, 0)

        # 手动曝光，曝光时间10ms
        mvsdk.CameraSetAeState(self.hCamera, 0)
        mvsdk.CameraSetExposureTime(self.hCamera, 5 * 1000)

        # 让SDK内部取图线程开始工作
        mvsdk.CameraPlay(self.hCamera)

        # 计算RGB buffer所需的大小，这里直接按照相机的最大分辨率来分配
        self.FrameBufferSize = self.cap.sResolutionRange.iWidthMax * self.cap.sResolutionRange.iHeightMax * (1 if self.monoCamera else 3)

        # 分配RGB buffer，用来存放ISP输出的图像
        # 备注：从相机传输到PC端的是RAW数据，在PC端通过软件ISP转为RGB数据（如果是黑白相机就不需要转换格式，但是ISP还有其它处理，所以也需要分配这个buffer）
        self.pFrameBuffer = mvsdk.CameraAlignMalloc(self.FrameBufferSize, 16)

        # 设置相机帧率
        self.timer = self.create_timer(1.0 / 60.0, self.timer_callback)

    def timer_callback(self):
        try:
            pRawData, FrameHead = mvsdk.CameraGetImageBuffer(self.hCamera, 200)
            mvsdk.CameraImageProcess(self.hCamera, pRawData, self.pFrameBuffer, FrameHead)
            mvsdk.CameraReleaseImageBuffer(self.hCamera, pRawData)

            # 此时图片已经存储在pFrameBuffer中，对于彩色相机pFrameBuffer=RGB数据，黑白相机pFrameBuffer=8位灰度数据
            # 把pFrameBuffer转换成opencv的图像格式以进行后续算法处理
            frame_data = (mvsdk.c_ubyte * FrameHead.uBytes).from_address(self.pFrameBuffer)
            frame = np.frombuffer(frame_data, dtype=np.uint8)
            frame = frame.reshape((FrameHead.iHeight, FrameHead.iWidth,
                                   1 if FrameHead.uiMediaType == mvsdk.CAMERA_MEDIA_TYPE_MONO8 else 3))

            frame = cv2.resize(frame, (1280, 1024), interpolation=cv2.INTER_LINEAR)

            result = self.find_white_circles_using_Contours(frame)
            if result is not None:
                unproj_pts, _ = self.cam.cam2world(result)
                point = Point()
                point.x = unproj_pts[0][0]
                point.y = unproj_pts[0][1]
                point.z = unproj_pts[0][2]

                # self.get_logger().info(f"Publishing Point: x={point.x}, y={point.y}, z={point.z}")
                angle = self.calculate_angle(point.x, point.z)
                # self.get_logger().info(f"Received message: x={point.x}, y={point.y}, z={point.z}")
                # self.get_logger().info(f"Angle in xoy plane: {angle:.2f} degrees")
                self.publisher_.publish(point)

            # perspective = self.cam.to_perspective(frame, (1024, 1280), 0.5)
            # cv2.imshow("Camera perspective", perspective)
            # cv2.waitKey(1)

        except mvsdk.CameraException as e:
            if e.error_code != mvsdk.CAMERA_STATUS_TIME_OUT:
                self.get_logger().error(f"CameraGetImageBuffer failed({e.error_code}): {e.message}")

    def find_white_circles_using_Contours(self, img):
        # 应用阈值化
        _, thresh = cv2.threshold(img, 100, 255, cv2.THRESH_BINARY)

        # 找到二值图像中的轮廓
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 初始化最大轮廓
        max_contour = max(contours, key=cv2.contourArea, default=None)

        if max_contour is not None:
            # 计算最大轮廓的中心
            M = cv2.moments(max_contour)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])

                # 绘制圆心
                cv2.circle(thresh, (cX, cY), 5, (0, 0, 255), -1)

                # 绘制最大轮廓
                cv2.drawContours(thresh, [max_contour], -1, (0, 255, 0), 2)

                # 显示结果
                cv2.imshow("Detected Max Contour", thresh)
                cv2.waitKey(1)

                return [cX, cY]
            else:
                return None
        else:
            return None
    
    def calculate_angle(self, x, z):
        angle_radians = np.arctan2(z, x)
        angle_degrees = np.degrees(angle_radians)
        return angle_degrees

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
