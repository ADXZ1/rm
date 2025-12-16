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
# 释放物体
print("释放物体...")
ret = robot.rm_set_gripper_release(
    speed=500,    # 释放速度
    block=True,   # 阻塞模式
    timeout=3     # 超时时间3秒
)
if ret != 0:
    raise Exception(f"释放物体失败，错误码：{ret}")
