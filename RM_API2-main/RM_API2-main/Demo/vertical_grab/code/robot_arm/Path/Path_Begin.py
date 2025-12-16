from Robotic_Arm.rm_robot_interface import *
import os
import time

# 设置Python脚本所在的路径
dst_path = "G:/rm/RM_API2-main/RM_API2-main/Demo/vertical_grab/code"
os.chdir(dst_path)  # 切换当前工作目录到指定路径

if __name__ == "__main__":
    # 初始化机械臂
    robot = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
    handle = robot.rm_create_robot_arm("192.168.2.18", 8080)

    # 获取API版本
    print("\nAPI 版本:", rm_api_version(), "\n")

    res = robot.rm_drag_trajectory_origin(1)