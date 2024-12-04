### This is ROS2 code used in fisheye camera to estiminate the position of IR led.
**1** Build
```
colcon build
```

**2** Open the UWB
```
ros2 launch nlink_parser_ros2 linktrack.launch.py
```
Check the data received by:
```
source install/setup.bash
ros2 topic echo /nlink_linktrack_nodeframe2
```

**3** Open the IR LED measure 
```
ros2 launch ir_led_measure ir_launch.py
```

**4** Open the ouster lidar to get the reflict data:
```
ros2 run reflict scan_node.py
```

**5** Get the rosbag
```
cd bags
ros2 bag record /fused_position
ros2 bag record /reflict_center
ros2 bag record /reflict_center /fused_position
```
View by the python code in bags

**7** Calibration
```
ros2 launch ir_led_measure rosbag_launch.py
ros2 bag record /camera/image_raw
```
