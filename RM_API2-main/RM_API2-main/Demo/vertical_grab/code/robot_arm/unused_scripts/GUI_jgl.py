import tkinter as tk
import subprocess
import os
import cv2
import pyrealsense2 as rs
import numpy as np
import socket
import threading
import pickle
import struct
import time
from PIL import Image, ImageTk
from loop import RobotController
from BaseCtr import RobotController as BSCTL
from class_ADP import ADP
import sys
from robot_server import RobotServer
from ultralytics import YOLO, SAM

dst_path = "/home/maic/rm/RM_API2-main/RM_API2-main/Demo/vertical_grab/vertical_grab/code"
os.chdir(dst_path)

# 初始化模型
yolo_model = YOLO("G:/rm/runs/train/bottle/weights/best.pt")
sam_model = SAM("G:/rm/RM_API2-main/RM_API2-main/Demo/vertical_grab/code/robot_arm/sam2.1_l.pt")

# 声明全局变量
baseCtl = BSCTL()
controller = None
robot1 = None
robot2 = None
robot_server = None

class CameraFrame(tk.Frame):

    def __init__(self, parent):
        super().__init__(parent, bg='#f8f9fa')
        self.parent = parent
        
        # 添加帧缓存和锁
        self.current_frame = None
        self.frame_lock = threading.Lock()
        self.current_depth_frame = None
        self.color_intr = None
        
        # 添加客户端连接状态标志
        self.client_connected = False
        self.client_lock = threading.Lock()
        
        # 创建标题
        self.title = tk.Label(self, text="实时预览", font=("Helvetica", 14, "bold"), 
                            bg='#f8f9fa', fg='#333')
        self.title.pack(pady=(5, 10))
        
        # 创建视频显示标签
        self.video_label = tk.Label(self)
        self.video_label.pack()
        
        # 初始化RealSense相机
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        
        # 配置流
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        
        try:
            # 启动相机流
            pipeline_profile = self.pipeline.start(self.config)
            
            # 获取相机内参
            color_stream = pipeline_profile.get_stream(rs.stream.color)
            color_intrinsics = color_stream.as_video_stream_profile().get_intrinsics()
            
            self.color_intr = {
                "ppx": color_intrinsics.ppx,
                "ppy": color_intrinsics.ppy,
                "fx": color_intrinsics.fx,
                "fy": color_intrinsics.fy
            }
            
            print("相机内参:", self.color_intr)
            
            # 等待稳定的图像
            time.sleep(1)
            
            self.is_running = True
            print("相机初始化成功")
            
            # 启动图像更新线程
            self.update_thread = threading.Thread(target=self.capture_frames, daemon=True)
            self.update_thread.start()
            
            # 启动GUI更新
            self.update_gui()
            
        except Exception as e:
            print(f"相机初始化失败: {e}")
            self.is_running = False
        
        # 启动socket服务器
        self.start_socket_server()
    
    def start_socket_server(self):
        """启动socket服务器"""
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind(('localhost', 12345))
        self.server_socket.listen(5)
        self.server_socket.setblocking(False)  # 设置为非阻塞模式
        
        # 在新线程中处理客户端连接
        self.server_thread = threading.Thread(target=self.handle_clients, daemon=True)
        self.server_thread.start()
    
    def handle_clients(self):
        """处理客户端连接的线程"""
        while self.is_running:
            try:
                client_socket, addr = self.server_socket.accept()
                print(f"客户端连接：{addr}")
                client_thread = threading.Thread(
                    target=self.handle_client_request,
                    args=(client_socket,),
                    daemon=True
                )
                client_thread.start()
            except BlockingIOError:
                # 非阻塞模式下，没有连接时会抛出此异常
                time.sleep(0.1)
            except Exception as e:
                print(f"处理客户端连接错误：{e}")
                time.sleep(0.1)
    
    def handle_client_request(self, client_socket):
        """处理单个客户端请求"""
        try:
            client_socket.setblocking(True)  # 设置为阻塞模式处理数据
            data = client_socket.recv(1024).decode()
            if data == "get_frames":
                with self.frame_lock:
                    if self.current_frame is not None and self.current_depth_frame is not None:
                        frames_data = {
                            'color': self.current_frame.copy(),  # 创建副本
                            'depth': self.current_depth_frame.copy(),
                            'intrinsics': self.color_intr
                        }
                        frames_packed = pickle.dumps(frames_data)
                        client_socket.sendall(struct.pack(">L", len(frames_packed)))
                        client_socket.sendall(frames_packed)
        except Exception as e:
            print(f"处理客户端请求错误：{e}")
        finally:
            client_socket.close()
    
    def capture_frames(self):
        """在独立线程中捕获图像"""
        while self.is_running:
            try:
                frames = self.pipeline.wait_for_frames()
                depth_frame = frames.get_depth_frame()
                color_frame = frames.get_color_frame()
                
                if not depth_frame or not color_frame:
                    continue
                
                depth_image = np.asanyarray(depth_frame.get_data())
                color_image = np.asanyarray(color_frame.get_data())
                
                with self.frame_lock:
                    self.current_frame = color_image
                    self.current_depth_frame = depth_image
                
            except Exception as e:
                print(f"捕获帧错误: {e}")
                time.sleep(0.03)
    
    def update_gui(self):
        """更新GUI显示"""
        try:
            with self.frame_lock:
                if self.current_frame is not None:
                    color_image_display = cv2.cvtColor(self.current_frame, cv2.COLOR_BGR2RGB)

                    image = Image.fromarray(color_image_display)
                    photo = ImageTk.PhotoImage(image=image)
                    self.video_label.configure(image=photo)
                    self.video_label.image = photo
        except Exception as e:
            print(f"更新GUI错误: {e}")
        
        self.parent.after(30, self.update_gui)
    
    def __del__(self):
        self.is_running = False
        if hasattr(self, 'pipeline'):
            self.pipeline.stop()
        if hasattr(self, 'server_socket'):
            self.server_socket.close()

def run_script(script_name):
    """在新线程中执行对应的 Python 脚本"""
    def run():
        try:
            subprocess.run(['python', script_name], check=True)
        except subprocess.CalledProcessError as e:
            print(f"Error occurred: {e}")
    
    # 创建新线程执行脚本
    thread = threading.Thread(target=run, daemon=True)
    thread.start()

def baseto0():
    def run_in_thread():
        global baseCtl
        if baseCtl is None:
            print("RobotController instance is not initialized.")
            return
        
        # 使用全局的 controller 实例移动到底盘0位
        baseCtl.move_to_position(0, 0)

    # 启动新线程运行机器人控制逻辑
    thread = threading.Thread(target=run_in_thread)
    thread.start()
    print("Started thread for moving to position (0, 0)")

def baseF0ToW1():
    def run_in_thread():
        global baseCtl
        if baseCtl is None:
            print("RobotController instance is not initialized.")
            return  
        # 使用全局的 controller 实例移动到底盘0位
        baseCtl.move_to_position(1, 0)
        time.sleep(1)
        baseCtl.move_slowly(0.2)
    # 启动新线程运行机器人控制逻辑
    thread = threading.Thread(target=run_in_thread)
    thread.start()
    print("Started thread for moving to position (0, 0)")
     
def baseFW1ToW2():
    def run_in_thread():
        global baseCtl
        if baseCtl is None:
            print("RobotController instance is not initialized.")
            return  
        # 使用全局的 controller 实例移动到底盘0位
        baseCtl.move_slowly(0)
        baseCtl.move_to_position(3, 1)
        time.sleep(1)
        baseCtl.move_slowly(0.22)
    # 启动新线程运行机器人控制逻辑
    thread = threading.Thread(target=run_in_thread)
    thread.start()
    print("Started thread for moving to position (0, 0)")

def baseFW2To0():
    def run_in_thread():
        global baseCtl
        if baseCtl is None:
            print("RobotController instance is not initialized.")
            return  
        # 使用全局的 controller 实例移动到底盘0位
        baseCtl.move_slowly(0.07)
        baseCtl.move_to_position(0, 0)
    # 启动新线程运行机器人控制逻辑
    thread = threading.Thread(target=run_in_thread)
    thread.start()
    print("Started thread for moving to position (0, 0)")
    
def baseF2To0():
    def run_in_thread():
        global baseCtl
        if baseCtl is None:
            print("RobotController instance is not initialized.")
            return  
        # 使用全局的 controller 实例移动到底盘0位
        baseCtl.move_to_position(0, 0)
# 启动GUI程序


# 创建 GUI 窗口
def create_gui():
    root = tk.Tk()
    root.title("机械臂操控系统")
    
    # 调整窗口大小和最小尺寸
    root.geometry('1366x900')
    root.minsize(1024, 768)
    root.configure(bg='#f0f2f5')

    # 专业配色方案
    PRIMARY_COLOR = '#2c3e50'
    SECONDARY_COLOR = '#3498db'
    SUCCESS_COLOR = '#27ae60'
    WARNING_COLOR = '#f39c12'
    DANGER_COLOR = '#e74c3c'
    LIGHT_BG = '#ecf0f1'
    DARK_TEXT = '#2c3e50'
    
    # 统一控件样式
    button_style = {
        'font': ('Microsoft YaHei', 10, 'bold'),
        'width': 14,
        'height': 1,
        'bg': SECONDARY_COLOR,
        'fg': 'white',
        'activebackground': '#2980b9',
        'relief': 'groove',
        'bd': 2,
        'padx': 5,
        'pady': 5
    }
    
    title_style = {
        'font': ('Microsoft YaHei', 12, 'bold'),
        'bg': LIGHT_BG,
        'fg': DARK_TEXT,
        'padx': 10,
        'pady': 5
    }
    
    frame_style = {
        'bg': LIGHT_BG,
        'padx': 10,
        'pady': 10,
        'highlightthickness': 1,
        'highlightbackground': '#bdc3c7'
    }

    # 创建主容器 - 使用网格布局
    main_frame = tk.Frame(root, bg=LIGHT_BG)
    main_frame.pack(fill='both', expand=True, padx=10, pady=10)
    
    # 配置网格权重
    main_frame.columnconfigure(0, weight=1)  # 左侧
    main_frame.columnconfigure(1, weight=3)   # 中间
    main_frame.columnconfigure(2, weight=1)  # 右侧
    main_frame.rowconfigure(0, weight=1)
    
    # 创建三列布局
    left_panel = tk.Frame(main_frame, **frame_style)
    left_panel.grid(row=0, column=0, sticky='nsew', padx=(0, 5))
    
    center_panel = tk.Frame(main_frame, bg='black')  # 黑色背景更适合视频
    center_panel.grid(row=0, column=1, sticky='nsew', padx=5)
    
    right_panel = tk.Frame(main_frame, **frame_style)
    right_panel.grid(row=0, column=2, sticky='nsew', padx=(5, 0))
    
    # 状态栏
    status_bar = tk.Frame(root, bg=PRIMARY_COLOR, height=24)
    status_bar.pack(fill='x', side='bottom')
    status_label = tk.Label(status_bar, text="系统就绪", fg='white', bg=PRIMARY_COLOR, font=('Microsoft YaHei', 9))
    status_label.pack(side='left', padx=10)
    
    # 在中间添加摄像头预览
    camera_frame = CameraFrame(center_panel)
    camera_frame.pack(expand=True, fill='both')
    
    # 辅助函数 - 创建带图标的标题
    def create_section(parent, title):
        frame = tk.Frame(parent, bg=LIGHT_BG)
        title_frame = tk.Frame(frame, bg=LIGHT_BG)
        tk.Label(title_frame, text=title, **title_style).pack(side='left')
        title_frame.pack(fill='x', pady=(0, 5))
        return frame
    
    # 辅助函数 - 创建按钮网格
    def create_button_grid(parent, buttons, columns=2):
        frame = tk.Frame(parent, bg=LIGHT_BG)
        for i, (text, script) in enumerate(buttons):
            row, col = divmod(i, columns)
            btn = tk.Button(frame, text=text, command=lambda s=script: run_script(s), **button_style)
            btn.grid(row=row, column=col, padx=5, pady=5, sticky='ew')
        return frame

    # 左侧面板 - 机械臂控制
    arm_control_frame = create_section(left_panel, "机械臂控制")
    arm_control_frame.pack(fill='x', pady=(0, 10))
    
    # 机械臂基本操作
    basic_buttons = [
        ("1、检测二维码", "0QRcode.py"),
        ("2、抓取瓶子", "1BottleGet.py"),
        ("4、抓回去", "grab_pingzi_ping1.py"),
        ("数据集采集", "dataset-capture.py"),
    ]
    create_button_grid(arm_control_frame, basic_buttons).pack(fill='x')
    
    # 移液控制
    pipette_frame = create_section(left_panel, "移液控制")
    pipette_frame.pack(fill='x', pady=(0, 10))
    
    pipette_buttons = [
        ("初始化", "init_ADP.py"),
        ("吸液", "ADP_xiye.py"),
        ("吐液", "ADP_tuye.py"),
    ]
    create_button_grid(pipette_frame, pipette_buttons, columns=2).pack(fill='x')
    
    # 轨迹控制
    path_frame = create_section(left_panel, "轨迹控制")
    path_frame.pack(fill='x')
    
    path_buttons = [
        ("录制轨迹", "Record_Path.py"),
        ("回轨迹起点", "Path_begin.py"),
    ]
    create_button_grid(path_frame, path_buttons).pack(fill='x')

    # 演示控制
    demo_frame = create_section(left_panel, "演示")
    demo_frame.pack(fill='x')
    
    demo_buttons = [
        ("机械臂归位", "spawn_robots.py"),
        ("抓取放置", "grab_and_place.py"),
        ("复现轨迹1", "Dragarm.py"),
        ("取用Y2", "2Tool2Quyong.py"),
        ("复现轨迹2", "Dragarm1.py"),
        ("吸液", "absorb_liquid.py"),
        ("复现轨迹3", "Dragarm2.py"),
        ("吐液", "dispense_liquid.py"),
        ("复现轨迹4", "Dragarm3.py"),
        ("放回Y2", "2Tool2Fanghui.py"),
        ("新动作","new_action1.py"),
        ("0-1-2-0","0-1-2-0.py"),
        ("UDP接收", "udp_recv.py")
    ]
    create_button_grid(demo_frame, demo_buttons).pack(fill='x')

    # 右侧面板 - 末端控制
    end_effector_frame = create_section(right_panel, "末端工具控制")
    end_effector_frame.pack(fill='x', pady=(0, 10))
    
    end_buttons = [
        ("打开夹爪", "release.py"),
        ("合上夹爪", "grab.py"),
        ("快换手解锁", "Kuaihuanshou_unlock.py"),
        ("快换手上锁", "Kuaihuanshou_lock.py")
    ]
    create_button_grid(end_effector_frame, end_buttons).pack(fill='x')
    
    # 锁枪控制
    lock_frame = create_section(right_panel, "锁枪控制")
    lock_frame.pack(fill='x', pady=(0, 10))
    
    lock_buttons = [
        ("取用Y1", "ToolQuyong_Y1.py"),
        ("放回Y1", "ToolFanghui_Y1.py"),
        ("取用Y2", "2Tool2Quyong.py"),
        ("放回Y2", "2Tool2Fanghui.py")
    ]
    create_button_grid(lock_frame, lock_buttons).pack(fill='x')
    
    # 继电器控制
    relay_frame = create_section(right_panel, "继电器控制")
    relay_frame.pack(fill='x')
    
    relay_buttons = [
        ("Y1锁紧", "Y1_open.py"),
        ("Y1解锁", "Y1_close.py"),
        ("Y2锁紧", "Y2_open.py"),
        ("Y2解锁", "Y2_close.py")
    ]
    create_button_grid(relay_frame, relay_buttons).pack(fill='x')

    def create_button_grid_2(parent, buttons,columns=2):
        frame = tk.Frame(parent, bg=LIGHT_BG)
        for i, (text, command) in enumerate(buttons):
            row, col = divmod(i, columns)
            btn = tk.Button(frame, text=text, command=command)
            btn.grid(row=row, column=col, padx=5, pady=5, sticky='ew')
        return frame
        # 继电器控制
    cfg_frame = create_section(right_panel, "机械臂配置")
    cfg_frame.pack(fill='x')
    
    robotcfg_buttons = [
        ("底盘0位", baseto0),
        ("底盘1位", baseF0ToW1),
        ("底盘2位", baseFW1ToW2),
        ("底盘2-0位", baseFW2To0),
    ]
    create_button_grid_2(cfg_frame, robotcfg_buttons).pack(fill='x')

    # 在关闭窗口时停止服务器
    def on_closing():
        print("关闭窗口，正在停止服务器...")
        if 'robot_server' in globals():
            robot_server.stop_server()
        root.destroy()
    
    root.protocol("WM_DELETE_WINDOW", on_closing)

    root.mainloop()

if __name__ == "__main__":
    # 初始化全局实例
    controller = RobotController()
    robot1 = controller.init_robot1()
    robot2 = controller.init_robot2()
    
    # 创建并启动机器人服务器
    print("正在启动机器人服务器...")
    robot_server = RobotServer()
    if not robot_server.start_server():
        print("警告: 机器人服务器启动失败，部分功能可能不可用")
    else:
        print("机器人服务器启动成功")
    
    create_gui()