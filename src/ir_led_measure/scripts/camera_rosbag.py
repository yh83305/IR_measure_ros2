#!/usr/bin/env python3
# coding=utf-8

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import mvsdk


class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_rosbag')
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()
        self.timer = self.create_timer(0.25, self.publish_image)  # Set timer for ~4 Hz (0.25s interval)

        print("start")

        # Open camera
        self.DevList = mvsdk.CameraEnumerateDevice()
        self.nDev = len(self.DevList)
        if self.nDev < 1:
            print("No camera was found!")
            return

        for i, DevInfo in enumerate(self.DevList):
            print(f"{i}: {DevInfo.GetFriendlyName()} {DevInfo.GetPortType()}")
        self.i = 0 if self.nDev == 1 else int(input("Select camera: "))
        self.DevInfo = self.DevList[self.i]
        print(self.DevInfo)

        # Initialize camera
        self.hCamera = 0
        try:
            self.hCamera = mvsdk.CameraInit(self.DevInfo, -1, -1)
        except mvsdk.CameraException as e:
            print(f"CameraInit Failed({e.error_code}): {e.message}")
            return

        # Get camera capabilities
        self.cap = mvsdk.CameraGetCapability(self.hCamera)
        self.monoCamera = (self.cap.sIspCapacity.bMonoSensor != 0)

        if self.monoCamera:
            mvsdk.CameraSetIspOutFormat(self.hCamera, mvsdk.CAMERA_MEDIA_TYPE_MONO8)
        else:
            mvsdk.CameraSetIspOutFormat(self.hCamera, mvsdk.CAMERA_MEDIA_TYPE_BGR8)

        # Set continuous capture mode
        mvsdk.CameraSetTriggerMode(self.hCamera, 0)

        # Set manual exposure time (30ms)
        mvsdk.CameraSetAeState(self.hCamera, 0)
        mvsdk.CameraSetExposureTime(self.hCamera, 50 * 1000)

        # Start internal image fetching thread
        mvsdk.CameraPlay(self.hCamera)

        # Allocate buffer for the maximum resolution
        self.FrameBufferSize = self.cap.sResolutionRange.iWidthMax * self.cap.sResolutionRange.iHeightMax * (1 if self.monoCamera else 3)
        self.pFrameBuffer = mvsdk.CameraAlignMalloc(self.FrameBufferSize, 16)

    def publish_image(self):
        if not rclpy.ok():
            return

        try:
            pRawData, FrameHead = mvsdk.CameraGetImageBuffer(self.hCamera, 200)
            mvsdk.CameraImageProcess(self.hCamera, pRawData, self.pFrameBuffer, FrameHead)
            mvsdk.CameraReleaseImageBuffer(self.hCamera, pRawData)

            frame_data = (mvsdk.c_ubyte * FrameHead.uBytes).from_address(self.pFrameBuffer)
            frame = np.frombuffer(frame_data, dtype=np.uint8)
            frame = frame.reshape((FrameHead.iHeight, FrameHead.iWidth,
                                   1 if FrameHead.uiMediaType == mvsdk.CAMERA_MEDIA_TYPE_MONO8 else 3))

            frame = cv2.resize(frame, (1280, 1024), interpolation=cv2.INTER_LINEAR)
            cv2.imshow("img", frame)
            cv2.waitKey(1)

            # Convert OpenCV image to ROS2 Image message
            encoding = 'mono8' if self.monoCamera else 'bgr8'
            image_msg = self.bridge.cv2_to_imgmsg(frame, encoding=encoding)

            # Set timestamp and frame ID
            image_msg.header.stamp = self.get_clock().now().to_msg()
            image_msg.header.frame_id = "camera_frame"

            # Publish image message
            self.image_pub.publish(image_msg)

        except mvsdk.CameraException as e:
            if e.error_code != mvsdk.CAMERA_STATUS_TIME_OUT:
                print(f"CameraGetImageBuffer failed({e.error_code}): {e.message}")

    def destroy_node(self):
        # Clean up camera resources on shutdown
        mvsdk.CameraUnInit(self.hCamera)
        mvsdk.CameraAlignFree(self.pFrameBuffer)
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    camera_publisher = CameraPublisher()
    try:
        rclpy.spin(camera_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        camera_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
