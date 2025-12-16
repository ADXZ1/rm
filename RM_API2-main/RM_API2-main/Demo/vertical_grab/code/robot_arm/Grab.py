import cv2
import time
import pyrealsense2 as rs
import numpy as np
from ultralytics import YOLO, SAM
from vertical_grab.interface import vertical_catch
from Robotic_Arm.rm_robot_interface import *

# 初始化机械臂
robot = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
handle = robot.rm_create_robot_arm("192.168.3.18", 8080)
# 夹取物体
print("夹取物体...")
ret = robot.rm_set_gripper_pick_on(
    speed=300,    # 释放速度
    force=50,    # 夹取力度 modby jgl 20210915
    block=True,   # 阻塞模式
    timeout=3     # 超时时间3秒
)
print(f"夹取结果：{ret}")
if ret != 0:
    raise Exception(f"夹取物体失败，错误码：{ret}")
