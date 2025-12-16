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

# 加载模型
yolo_model = YOLO("G:/rm/runs/train/open13/weights/best.pt")
sam_model = SAM("sam2.1_l.pt")

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

# 夹取试管后的放置函数
def place_at_fixed_position(robot):
    """
    固定位置放置
    :param robot: 机械臂对象
    """
    try:
        # 1. 放置的预备位置（在放置点上方）
        place_above_pose = PLACE_POSITION['above'].copy()
        print("放置位置：", place_above_pose)
        #bx = 0.023
        #by = 0.007
        #place_above_pose[0]+=bx
        #place_above_pose[1]+=by
        #print("放置位置：", place_above_pose)

        
        ret = robot.rm_movej_p(place_above_pose, v=MOVE_SPEED, r=0, connect=0, block=1)
        if ret != 0:
            raise Exception(f"移动失败，错误码：{ret}")
        
        # 2. 下降到放置高度
        place_above_pose[2] -= PLACE_POSITION['drop_height']
        ret = robot.rm_movel(place_above_pose, v=MOVE_SPEED, r=0, connect=0, block=1)
        if ret != 0:
            raise Exception(f"移动失败，错误码：{ret}")

        # 3. 释放物体
        attempts = 0
        while attempts < MAX_ATTEMPTS:
            print("释放物体...")
            ret = robot.rm_set_gripper_release(
                speed=GRIPPER_CONFIG['release']['speed'],
                block=True,
                timeout=GRIPPER_CONFIG['release']['timeout']
            )

            if ret == 0:
                print("释放成功")
                break
            else:
                print(f"释放失败，错误码：{ret}")
                attempts += 1
                time.sleep(1)

        if attempts == MAX_ATTEMPTS:
            print("达到最大尝试次数，释放操作仍未成功")
        
        # 4. 抬升回到预备位置
        print("抬升回到预备位置...")
        place_above_pose[2] += PLACE_POSITION['drop_height']
        ret = robot.rm_movel(place_above_pose, v=MOVE_SPEED, r=0, connect=0, block=1)
        if ret != 0:
            raise Exception(f"抬升失败，错误码：{ret}")
        initial_pose = [-0.303379, 0.274441, -0.075986, -3.081, 0.137, -1.828]

        #移动到初始位置
        print("移动到初始位置...")
        ret = robot.rm_movej_p(initial_pose, v=20, r=0, connect=0, block=1)
        if ret != 0:
            raise Exception(f"移动到最终位置失败，错误码：{ret}")
        return True
        
    except Exception as e:
        print(f"放置过程出错: {str(e)}")
        return False

# GMM处理函数
def process_mask_with_gmm(image, mask, n_components=1):
    """
    使用高斯混合模型(GMM)处理图像分割掩码
    """
    # 提取掩码区域对应的图像特征
    masked_image = cv2.bitwise_and(image, image, mask=mask)
    # 获取非零像素点的坐标和颜色值
    y_coords, x_coords = np.nonzero(mask)
    pixels = masked_image[y_coords, x_coords]
    
    if len(pixels) == 0:
        return mask

    # 构建特征向量（包含位置和颜色信息）
    features = np.column_stack((x_coords, y_coords, pixels))
    
    # 训练GMM模型
    gmm = GaussianMixture(n_components=n_components, random_state=42)
    labels = gmm.fit_predict(features)
    
    # 创建新的掩码
    new_mask = np.zeros_like(mask)
    # 选择最大的连通区域
    for i in range(n_components):
        component_mask = np.zeros_like(mask)
        component_indices = (labels == i)
        component_mask[y_coords[component_indices], x_coords[component_indices]] = 255
        
        # 使用连通区域分析
        num_labels, labels_im = cv2.connectedComponents(component_mask)
        if num_labels > 1:
            # 找出最大的连通区域
            largest_label = 1 + np.argmax([np.sum(labels_im == i) for i in range(1, num_labels)])
            component_mask = (labels_im == largest_label).astype(np.uint8) * 255
        
        new_mask = cv2.bitwise_or(new_mask, component_mask)
    
    # 应用形态学操作改善掩码质量
    kernel = np.ones((5,5), np.uint8)
    new_mask = cv2.morphologyEx(new_mask, cv2.MORPH_CLOSE, kernel)
    new_mask = cv2.morphologyEx(new_mask, cv2.MORPH_OPEN, kernel)
    
    return new_mask

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
def capture_and_move(robot, width=640, height=480, fps=60):
    """获取一帧图像并执行移动"""
    try:
        ret = robot.rm_set_gripper_release(
                speed=100,    # 释放速度
                block=True,   # 阻塞模式
                timeout=3     # 超时时间3秒
            )
        # 记录初始位姿
        ret, initial_state = robot.rm_get_current_arm_state()
        if ret != 0:
            raise Exception(f"获取初始位姿失败，错误码：{ret}")
        
        initial_pose = initial_state['pose']
        print("\n==== 初始位姿 ====")
        print("初始位姿:", initial_pose)

        # 获取彩色图像、深度图像和相机内参
        color_image, depth_image, color_intr = get_frames_from_gui()
        if color_image is None or depth_image is None or color_intr is None:
            raise Exception("无法获取图像或相机内参")
        
        # 保存原始图像
        cv2.imwrite('G:/rm/RM_API2-main/RM_API2-main/Demo/vertical_grab/pictures/original_image.jpg', 
                    cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR))
        
        # YOLO检测获取边界框
        yolo_results = yolo_model(color_image, verbose=False)
        
        # 创建掩码图像
        mask = np.zeros((height, width), dtype=np.uint8)

        #print(yolo_results)
        
        # 处理检测结果
        detected = False
        for result in yolo_results:
            boxes = result.boxes
            for box in boxes:
                print(box)
                confidence = float(box.conf)
                if confidence < 0.7:
                    continue

                detected = True
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                bbox = [int(x1), int(y1), int(x2), int(y2)]
                
                # 使用SAM进行初始分割
                sam_results = sam_model(color_image, bboxes=[bbox])
                
                if sam_results and len(sam_results) > 0:
                    sam_mask = sam_results[0].masks.data[0].cpu().numpy()
                    sam_mask = (sam_mask * 255).astype(np.uint8)
                    
                    # 使用GMM改进分割结果
                    improved_mask = process_mask_with_gmm(color_image, sam_mask)
                    
                    # 更新掩码
                    mask = cv2.bitwise_or(mask, improved_mask)
                    
                    # 保存改进后的掩码用于调试
                    cv2.imwrite('G:/rm/RM_API2-main/RM_API2-main/Demo/vertical_grab/pictures/improved_mask.jpg', 
                              improved_mask)
                
                # 在原图上显示检测框和轮廓
                cv2.rectangle(color_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)

        if not detected:
            cv2.imwrite('G:/rm/RM_API2-main/RM_API2-main/Demo/vertical_grab/pictures/failed_detection.jpg', 
                       cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR))
            raise Exception("未检测到目标")

        # 保存结果图像
        cv2.imwrite('G:/rm/RM_API2-main/RM_API2-main/Demo/vertical_grab/pictures/mask_result.jpg', mask)

        # 获取机械臂当前状态
        ret, state_dict = robot.rm_get_current_arm_state()
        if ret != 0:
            raise Exception(f"获取机械臂状态失败，错误码：{ret}")
        
        pose = state_dict['pose']
        print("\n==== 当前状态 ====")
        print("当前位姿:", pose)

        # 1. 第一次计算目标位姿
        above_object_pose, correct_angle_pose, finally_pose  = vertical_catch(
            mask, 
            depth_image, 
            color_intr, 
            pose,  
            100,   
            gripper_offset,
            rotation_matrix, 
            translation_vector
        )

        # 2. 移动到预备位置（让相机对准物体上方）
        print("移动到预备位置...")
        camera_above_pose = above_object_pose.copy()
        camera_above_pose[0] -= 0.08
        # camera_above_pose[1] -= 0.05
        
        print("\n==== 移动详细信息 ====")
        print(f"目标位姿: {camera_above_pose}")
        print(f"位置变化: dx={camera_above_pose[0]-pose[0]:.3f}, "
              f"dy={camera_above_pose[1]-pose[1]:.3f}, "
              f"dz={camera_above_pose[2]-pose[2]:.3f}")

        ret = robot.rm_movej_p(camera_above_pose, v=15, r=0, connect=0, block=1)
        if ret != 0:
            raise Exception(f"移动到预备位置失败，错误码：{ret}")
        time.sleep(1)

         # 3. 在相机正对物体时进行二次检测
        print("\n==== 二次检测 ====")
        color_image, depth_image, color_intr = get_frames_from_gui()
        if color_image is None or depth_image is None or color_intr is None:
            raise Exception("二次检测：无法获取图像或相机内参")
        
        # 二次YOLO检测和SAM分割
        yolo_results = yolo_model(color_image, verbose=False)
        mask = np.zeros((height, width), dtype=np.uint8)
        
        detected = False
        for result in yolo_results:
            boxes = result.boxes
            for box in boxes:
                confidence = float(box.conf)
                if confidence < 0.7:
                    continue
                detected = True
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                bbox = [int(x1), int(y1), int(x2), int(y2)]
                
                sam_results = sam_model(color_image, bboxes=[bbox])
                if sam_results and len(sam_results) > 0:
                    sam_mask = sam_results[0].masks.data[0].cpu().numpy()
                    sam_mask = (sam_mask * 255).astype(np.uint8)
                    
                    # 使用GMM改进二次检测的分割结果
                    improved_mask = process_mask_with_gmm(color_image, sam_mask)
                    
                    # 更新掩码
                    mask = cv2.bitwise_or(mask, improved_mask)
                    
                    # 保存二次检测改进后的掩码用于调试
                    cv2.imwrite('G:/rm/RM_API2-main/RM_API2-main/Demo/vertical_grab/pictures/second_improved_mask.jpg', improved_mask)

        if not detected:
            raise Exception("二次检测：未检测到目标")

        # 获取当前机械臂位姿
        ret, current_state = robot.rm_get_current_arm_state()
        if ret != 0:
            raise Exception(f"获取当前状态失败，错误码：{ret}")
        current_pose = current_state['pose']

        # 4. 基于二次检测结果计算新的位姿和下降距离
        _, adjusted_angle_pose, adjusted_final_pose = vertical_catch(
            mask, 
            depth_image,  
            color_intr, 
            current_pose,
            100,
            gripper_offset,
            rotation_matrix, 
            translation_vector
        )

        # 5. 执行抓取动作序列
        print("\n==== 开始移动 ====")
        
        adjusted_final_pose[3] = gripper_offset[0]
        adjusted_final_pose[4] = gripper_offset[1]
        adjusted_final_pose[5] = gripper_offset[2]

        # 先移动到目标物体正上方（保持当前Z高度）
        print("移动到目标物体正上方...")
        above_target_pose = adjusted_final_pose.copy()
        above_target_pose[2] = current_pose[2]  # 保持当前Z高度
        above_target_pose[1] -= 0.015  # 稍微向左移动一点

        print("\n==== XY平面移动详细信息 ====")
        print(f"当前位姿: {current_pose}")
        print(f"目标上方位姿: {above_target_pose}")
        ret = robot.rm_movel(above_target_pose, v=15, r=0, connect=0, block=1)
        if ret != 0:
            raise Exception(f"移动到目标物体上方失败，错误码：{ret}")
        
        # 垂直下降到抓取位置
        print("垂直下降到抓取位置...")
        print("\n==== Z轴下降详细信息 ====")
        print(f"目标位姿: {adjusted_final_pose}")
        adjusted_final_pose[1] -= 0.015
        adjusted_final_pose[2] = -0.24#-0.195
        ret = robot.rm_movel(adjusted_final_pose, v=15, r=0, connect=0, block=1)  # 降低速度进行精确抓取
        if ret != 0:
            raise Exception(f"垂直下降失败，错误码：{ret}")

        # 定义最大尝试次数
        max_attempts = 5
        attempts = 0

        while attempts < max_attempts:
            print("夹取物体...")
            ret = robot.rm_set_gripper_pick_on(
                speed=100,    # 夹取速度
                block=True,   # 阻塞模式
                timeout=3,    # 超时时间3秒
                force=300     # 力度100
            )

            if ret == 0:
                print("夹取成功")
                break
            else:
                print(f"夹取失败，错误码：{ret}")
                attempts += 1
                time.sleep(1)  

        if attempts == max_attempts:
            print("达到最大尝试次数，夹取操作仍未成功")

        # 回到初始位姿
        print("回到初始位姿...")
        ret = robot.rm_movej_p(initial_pose, v=15, r=0, connect=0, block=1)
        if ret != 0:
            raise Exception(f"回到初始位姿失败，错误码：{ret}")

        if ret != 0:
            raise Exception(f"释放物体失败，错误码：{ret}")

        # 获取最终状态
        ret, final_state = robot.rm_get_current_arm_state()
        if ret == 0:
            print("\n==== 移动完成 ====")
            print("最终位姿:", final_state['pose'])
        
        return True

    except Exception as e:
        print(f"错误: {str(e)}")
        return False

# 取回试管函数
def return_bottle_to_original(robot):
    """
    把瓶子从放置位置拿回原位（抓取时的位置）
    :param robot: 机械臂对象
    """
    try:
        # 1. 移动到放置点上方
        place_above_pose = PLACE_POSITION['above'].copy()
        ret = robot.rm_movej_p(place_above_pose, v=MOVE_SPEED, r=0, connect=0, block=1)
        if ret != 0:
            raise Exception(f"移动到放置点上方失败，错误码：{ret}")
        
        # 2. 下降到夹取高度
        place_above_pose[2] -= PLACE_POSITION['drop_height']
        ret = robot.rm_movel(place_above_pose, v=MOVE_SPEED, r=0, connect=0, block=1)
        if ret != 0:
            raise Exception(f"下降到夹取高度失败，错误码：{ret}")
        
        # 3. 夹取瓶子
        attempts = 0
        while attempts < MAX_ATTEMPTS:
            print("夹取物体...")
            ret = robot.rm_set_gripper_pick_on(
                speed=GRIPPER_CONFIG['pick']['speed'],
                block=True,
                timeout=GRIPPER_CONFIG['pick']['timeout'],
                force=GRIPPER_CONFIG['pick']['force']
            )
            if ret == 0:
                print("夹取成功")
                break
            else:
                print(f"夹取失败，错误码：{ret}")
                attempts += 1
                time.sleep(1)
        
        if attempts == MAX_ATTEMPTS:
            raise Exception("达到最大尝试次数，夹取操作仍未成功")
        
        # 4. 抬升到安全高度
        place_above_pose = PLACE_POSITION['above'].copy()
        ret = robot.rm_movel(place_above_pose, v=MOVE_SPEED, r=0, connect=0, block=1)
        if ret != 0:
            raise Exception(f"抬升到安全高度失败，错误码：{ret}")
        
        # 5. 移动到抓取位置上方
        grab_above_pose = GRAB_POSITION['above'].copy()
        ret = robot.rm_movej_p(grab_above_pose, v=MOVE_SPEED, r=0, connect=0, block=1)
        if ret != 0:
            raise Exception(f"移动到抓取位置上方失败，错误码：{ret}")
        
        # 6. 下降到释放高度
        grab_above_pose[2] -= GRAB_POSITION['drop_height']
        ret = robot.rm_movel(grab_above_pose, v=MOVE_SPEED, r=0, connect=0, block=1)
        if ret != 0:
            raise Exception(f"下降到释放高度失败，错误码：{ret}")
        
        # 7. 释放瓶子
        attempts = 0
        while attempts < MAX_ATTEMPTS:
            print("释放物体...")
            ret = robot.rm_set_gripper_release(
                speed=GRIPPER_CONFIG['release']['speed'],
                block=True,
                timeout=GRIPPER_CONFIG['release']['timeout']
            )
            if ret == 0:
                print("释放成功")
                break
            else:
                print(f"释放失败，错误码：{ret}")
                attempts += 1
                time.sleep(1)
        
        if attempts == MAX_ATTEMPTS:
            raise Exception("达到最大尝试次数，释放操作仍未成功")
        
        # 8. 抬升到安全高度
        grab_above_pose[2] += GRAB_POSITION['drop_height']
        ret = robot.rm_movel(grab_above_pose, v=MOVE_SPEED, r=0, connect=0, block=1)
        if ret != 0:
            raise Exception(f"抬升到安全高度失败，错误码：{ret}")
        
        # 9. 回到初始姿态
        ret = robot.rm_movej_p(INITIAL_POSE, v=MOVE_SPEED, r=0, connect=0, block=1)
        if ret != 0:
            raise Exception(f"回到初始姿态失败，错误码：{ret}")
        
        return True
        
    except Exception as e:
        print(f"返回瓶子过程出错: {str(e)}")
        return False

if __name__ == "__main__":
    try:
        success = capture_and_move(robot)
        if success:
            print("抓取执行完成")
            
            # 放置物体
            if place_at_fixed_position(robot):
                print("放置完成")
                
                # 等待一段时间后，将瓶子放回原位
                print("等待5秒后开始将瓶子放回原位...")
                time.sleep(5)
                
                if return_bottle_to_original(robot):
                    print("瓶子已成功放回原位")
                else:
                    print("瓶子放回原位失败")
                
            # 回到初始位姿
            print("\n==== 回到初始位姿 ====")
            initial_pose = [-0.303379, 0.274441, -0.075986, -3.081, 0.137, -1.828]
            ret = robot.rm_movej_p(initial_pose, v=15, r=0, connect=0, block=1)
            if ret != 0:
                raise Exception(f"回到初始位姿失败，错误码：{ret}")
            
            print("任务完成")
        else:
            print("放置失败")
            
    except Exception as e:
        print(f"错误: {str(e)}")