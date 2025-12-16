import cv2
import time
import numpy as np
from ultralytics import YOLO, SAM
from vertical_grab.interface import vertical_catch
from Robotic_Arm.rm_robot_interface import *
import os
from sklearn.mixture import GaussianMixture
import socket
import pickle
import struct
from config import *  # 导入配置

# 设置Python脚本所在的路径
dst_path = "G:/rm/RM_API2-main/RM_API2-main/Demo/vertical_grab/code/robot_arm/"
os.chdir(dst_path)

# 初始化机械臂
robot = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
handle = robot.rm_create_robot_arm("192.168.3.18", 8080)


# 手眼标定参数
rotation_matrix = [[0.013632, 0.99971, 0.019696], 
                    [-0.99988, 0.013487, 0.0075167],
                    [0.007249, -0.019796, 0.99978]]
# 使用侧装的姿态角
gripper_offset = [3.146, 0, 3.128]  # 侧装的垂直姿态
translation_vector = [-0.07797346114669192, 0.0607491485688067, -0.09132410501940941]

# 在开始移动前检查机械臂状态
ret, state = robot.rm_get_current_arm_state()
if ret != 0:
    raise Exception(f"获取机械臂状态失败，错误码：{ret}")

if state.get('error_code', 0) != 0:
    raise Exception(f"机械臂存在错误，错误码：{state['error_code']}")

print(f"机械臂当前状态: {state}")

QRinitialPOSE = LEFT_INITIAL_POSE
biaspose_bottle = [-0.15,-0.1,0,0,0,0]
biaspose_plate = [0.15,0.1,0,0,0,0]

def move_to_bias_based_onQRcode(robot, bias_pose,m_QRinitialPOSE = QRinitialPOSE):
    curpose = m_QRinitialPOSE #def move_to_bias_based_onQRcode(robot, bias_pose,QRinitialPOSE = QRinitialPOSE):
    print(bias_pose)
    newpose = [curpose[0]+bias_pose[0], curpose[1]+bias_pose[1], curpose[2]+bias_pose[2], curpose[3]+bias_pose[3], curpose[4]+bias_pose[4], curpose[5]+bias_pose[5]]
    ret = gotoinitialpose(robot,newpose)
    return ret

def move_to_bias(robot, bias_pose):
    # 在开始移动前检查机械臂状态
    ret, state = robot.rm_get_current_arm_state()
    if ret != 0:
        raise Exception(f"获取机械臂状态失败，错误码：{ret}")

    if state.get('error_code', 0) != 0:
        raise Exception(f"机械臂存在错误，错误码：{state['error_code']}")

    print(f"机械臂当前状态: {state}")
    curpose = state['pose']
    newpose = [curpose[0]+bias_pose[0], curpose[1]+bias_pose[1], curpose[2]+bias_pose[2], curpose[3]+bias_pose[3], curpose[4]+bias_pose[4], curpose[5]+bias_pose[5]]
    ret = gotoinitialpose(robot,newpose)
    return ret

def gotoinitialpose(robot,pose):
    try:
        print("qrcode_grab: 移动到初始位置")
        ret = robot.rm_movej_p(pose, v=MOVE_SPEED, r=0, connect=0, block=1)
        print(ret)
        time.sleep(1)

        if ret != 0:
            raise Exception(f"移动失败，错误码：{ret}")
        else:
            return True
    except Exception as e:
        print(f"过程出错: {str(e)}")
        return False  


# 调用摄像头函数
def get_frames_from_gui(max_retries=3, timeout=5):
    retries = 0
    while retries < max_retries:
        try:
            # 创建socket客户端
            client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client_socket.settimeout(timeout)
            client_socket.connect(('localhost', 12345))
            
            # 发送获取帧的请求
            client_socket.send("get_frames".encode())
            
            # 接收数据大小
            data_size = struct.unpack(">L", client_socket.recv(4))[0]
            
            # 接收图像数据
            received_data = b""
            while len(received_data) < data_size:
                data = client_socket.recv(4096)
                if not data:
                    break
                received_data += data
            
            # 反序列化图像数据
            frames_data = pickle.loads(received_data)
            return frames_data['color'], frames_data['depth'], frames_data['intrinsics']
        
        except (socket.timeout, ConnectionRefusedError) as e:
            print(f"连接尝试 {retries + 1} 失败: {e}")
            retries += 1
            time.sleep(1)  # 等待1秒后重试
        except Exception as e:
            print(f"获取帧错误：{e}")
            return None, None, None
        finally:
            client_socket.close()
    
    print("达到最大重试次数，无法获取帧")
    return None, None, None

# 视觉定位函数
def capture_QRcode(robot, width=640, height=480, fps=60):
    color_image, depth_image, color_intr = get_frames_from_gui()
   # print(color_intr)
   # print("get image success")

    if color_image is None or depth_image is None or color_intr is None:
        raise Exception("无法获取图像或相机内参")
    rst = detect_qr_code(color_image, depth_image, color_intr)
    return rst

def order_points(pts):
    """将四个角点按左上、右上、右下、左下顺序排列"""
    rect = np.zeros((4, 2), dtype="float32")
    s = pts.sum(axis=1)
    rect[0] = pts[np.argmin(s)]  # 左上角（最小的x+y）
    rect[2] = pts[np.argmax(s)]  # 右下角（最大的x+y）
    diff = np.diff(pts, axis=1)
    rect[1] = pts[np.argmin(diff)]  # 右上角（最小的x-y）
    rect[3] = pts[np.argmax(diff)]  # 左下角（最大的x-y）
    return rect

def rotation_angle(rect):
    """计算二维码相对于水平轴的旋转角度（度）"""
    # 计算左上到右上的向量
    vec = rect[1] - rect[0]
    angle_rad = np.arctan2(vec[1], vec[0])
    angle_deg = np.degrees(angle_rad)
    return angle_deg

def detect_qr_code(color_image, depth_image, color_intr):
    gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
    detector = cv2.QRCodeDetector()
    
    # 检测并解码
    decoded_info, points, _ = detector.detectAndDecode(gray)
    
    if decoded_info and points is not None:
        # 确保points是4x2数组
        points = points.astype(np.float32)
        if points.shape != (4, 2):
            points = points.reshape((4, 2))
        
        # 排序角点
        rect = order_points(points)
        (tl, tr, br, bl) = rect
        
        # 计算旋转角度
        angle = rotation_angle(rect)
        
        # 计算中心坐标
        center_x = int((tl[0] + br[0]) / 2)
        center_y = int((tl[1] + br[1]) / 2)
        
        # 绘制结果
        color_vis = color_image.copy()
        cv2.polylines(color_vis, [rect.astype(np.int32)], True, (0,255,0), 2)
        cv2.circle(color_vis, (center_x, center_y), 5, (0,0,255), -1)
        cv2.putText(color_vis, f"Angle: {angle:.2f}deg", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)
        
        # 计算空间位姿
        fx = color_intr['fx']
        fy = color_intr['fy']
        cx = color_intr['ppx']
        cy = color_intr['ppy']
        
        #depth = depth_image[center_y, center_x]
        depth = get_robust_depth(depth_image, center_x, center_y, window_size=5)
        if depth is not None:
            Z = depth / 1000.0  # 毫米转米
            X = (center_x - cx) * Z / fx
            Y = (center_y - cy) * Z / fy
            
            # 计算欧拉角（绕Z轴的旋转角）
            # 通过旋转矩阵转换
            obj_pts = np.array([[-0.05, -0.05, 0],
                               [0.05, -0.05, 0],
                               [0.05, 0.05, 0],
                               [-0.05, 0.05, 0]], dtype=np.float32)
            
            # 使用solvePnP计算位姿
            _, rvec, tvec = cv2.solvePnP(obj_pts, rect, 
                                         np.array([[fx,0,cx],[0,fy,cy],[0,0,1]]),
                                         np.zeros((4,1)))
            R, _ = cv2.Rodrigues(rvec)
            yaw = np.degrees(np.arctan2(R[1,0], R[0,0]))  # 绕Z轴的旋转角
            
            cv2.putText(color_vis, f"X={X:.3f}m, Y={Y:.3f}m, Z={Z:.3f}m", (10, 60),cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)
           # print(f"空间位姿：X={X:.3f}m, Y={Y:.3f}m, Z={Z:.3f}m")
           # print(f"旋转角度：{angle:.1f}°（像素坐标系）, {yaw:.1f}°（相机坐标系绕Z轴）")
            CX, CY, CA = calculate_camera_position_in_marker_frame(X, Y, angle)
            cv2.putText(color_vis, f"X1={CX:.3f}m, Y1={CY:.3f}m, Z1={-Z:.3f}m, A1={CA:.1f} deg", (10, 90),cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)        
            #print(f"空间位姿：X={X:.3f}m, Y={Y:.3f}m, Z={Z:.3f}m")
            #print(f"旋转角度：{angle:.1f}°（像素坐标系）, {yaw:.1f}°（相机坐标系绕Z轴）")
            print(f"相机坐标系：X1={CX:.3f}m, Y1={CY:.3f}m, Z1={-Z:.3f}m, A1={CA:.1f} deg")
            return True,CX, CY, CA,Z
        else:
            print("深度无效，无法计算空间位姿")
            
        cv2.imshow("QR Detection", color_vis)
        cv2.waitKey(10)
        return False,None,None,None,None
    else:
        print("未检测到有效二维码")
        cv2.imshow("QR Detection", color_image)
        return False,None,None,None,None



import math
def calculate_camera_position_in_marker_frame(X_m, Y_m, a_m):
            # 将角度从度数转换为弧度
    a_m_rad = math.radians(a_m)          
            # 计算摄像头在 marker 坐标系中的位置
    X_c_prime = -(X_m * math.cos(a_m_rad) + Y_m * math.sin(a_m_rad))
    Y_c_prime = -(Y_m * math.cos(a_m_rad) - X_m * math.sin(a_m_rad))         
            # 计算摄像头在 marker 坐标系中的角度
    a_c_prime = -a_m            
    return X_c_prime, Y_c_prime, a_c_prime

def get_robust_depth(depth_image, center_x, center_y, window_size=5, min_valid=3):
    """
    在中心点周围区域计算鲁棒的深度值（过滤无效值后取均值或中位数）

    参数:
        depth_image: 深度图像数组（单位：毫米）
        center_x: 中心点x坐标
        center_y: 中心点y坐标
        window_size: 区域边长（必须为奇数）
        min_valid: 最小有效像素数（否则返回None）

    返回:
        depth_value: 鲁棒的深度值（米）或 None（无效）
    """
    # 确保window_size为奇数
    if window_size % 2 == 0:
        window_size += 1

    # 计算区域边界
    pad = window_size // 2
    y_min = max(0, center_y - pad)
    y_max = min(depth_image.shape[0], center_y + pad + 1)
    x_min = max(0, center_x - pad)
    x_max = min(depth_image.shape[1], center_x + pad + 1)

    # 提取区域深度值
    region = depth_image[y_min:y_max, x_min:x_max].flatten()
    
    # 过滤无效值（0或过小值）
    valid_depths = region[(region > 10)]  # 假设有效深度大于10mm

    if len(valid_depths) >= min_valid:
        # 返回中位数（更鲁棒）或均值
        return -1*np.median(valid_depths)  # 转换为米
    else:
        return None
    

def save_initial_pose_to_file(file_path, pose):
    """
    将初始位置保存到指定的 txt 文件中。
    :param file_path: 文件路径
    :param pose: 要保存的初始位置列表
    """
    try:
        with open(file_path, 'w') as file:  # 打开文件并清空内容
            file.write(str(pose))  # 将列表转换为字符串并写入文件
        print(f"初始位置已成功保存到 {file_path}")
    except Exception as e:
        print(f"保存初始位置时出错: {e}")


import ast
import json
def load_initial_pose_from_file(file_path):
    """
    从指定的 txt 文件中读取初始位置。
    :param file_path: 文件路径
    :return: 初始位置列表或 None（如果读取失败）
    """
    try:
        with open(file_path, 'r') as file:  # 打开文件进行读取
            content = file.read().strip()  # 去除可能存在的空白字符
            pose = ast.literal_eval(content)  # 安全地将字符串转换回列表
            if isinstance(pose, list):  # 确保读取的内容是列表
                print(f"从 {file_path} 成功加载初始位置")
                return pose
            else:
                print("文件内容不是有效的列表格式")
                return None
    except Exception as e:
        print(f"读取初始位置时出错: {e}")
        return None


def get_bias(robot,basepose):
    # 在开始移动前检查机械臂状态
    ret, state = robot.rm_get_current_arm_state()
    if ret != 0:
        raise Exception(f"获取机械臂状态失败，错误码：{ret}")

    if state.get('error_code', 0) != 0:
        raise Exception(f"机械臂存在错误，错误码：{state['error_code']}")

    print(f"机械臂当前状态: {state}")
    curpose = state['pose']
    bias = [curpose[i] - basepose[i] if i < 3 else normalize_angle(curpose[i] - basepose[i])
             for i in range(6)]
    #= basepose - curpose
    print(f"初始位置偏差：{bias}")
    return bias

def normalize_angle(angle):
    """
    将角度归一化到 [-π, π] 范围内。
    :param angle: 输入的角度（弧度）
    :return: 归一化后的角度（弧度）
    """
    return (angle + math.pi) % (2 * math.pi) - math.pi

from datetime import datetime
if __name__ == "__main__":
    try:
        QR_INIT_POSE = load_initial_pose_from_file('0QRcode_initial_pose.txt')
        if QR_INIT_POSE is None:
            exit()
        else:
            print(f"初始位置：{QR_INIT_POSE}")
            get_bias(robot,QR_INIT_POSE)
               
    except Exception as e:
        print(f"错误: {str(e)}")