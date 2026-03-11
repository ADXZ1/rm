import os
import json

# 配置文件路径
POSITION_CONFIG_FILE = os.path.join(os.path.dirname(__file__), 'position_config.txt')

def update_offset(current_x, current_y, current_angle=0):
    """
    更新当前偏移量并保存到文件
    :param current_x: 当前x位置（厘米）
    :param current_y: 当前y位置（厘米）
    :param current_angle: 当前角度
    """
    try:
        # 读取现有配置
        with open(POSITION_CONFIG_FILE, 'r', encoding='utf-8') as f:
            config = json.load(f)
        
        # 更新偏移量
        config['current_offset']['x'] = current_x
        config['current_offset']['y'] = current_y
        config['current_offset']['angle'] = current_angle
        
        # 写回文件
        with open(POSITION_CONFIG_FILE, 'w', encoding='utf-8') as f:
            json.dump(config, f, indent=4)
            
    except Exception as e:
        print(f"更新位置配置文件失败: {str(e)}")

# 初始位置
INITIAL_POSE = [-0.303379, 0.274441, -0.075986, -3.081, 0.137, -1.828]


# for qr code detection
LEFT_INITIAL_POSE_zero = [-0.007, 0.207, -0.1547, 3.101, 0.111, 3.144]
LEFT_INITIAL_POSE = [-0.356, 0.309, -0.186, -3.141, 0, -1.89]


RIGHT_INITIAL_POSE_zero = [0.007, 0.207, -0.1547, 3.101, 0.111, 3.144]
RIGHT_INITIAL_POSE = [-0.372, 0.221, -0.186, -3.121, 0, -1.89]


# 夹爪配置
GRIPPER_CONFIG = {
    'pick': {
        'speed': 200,    # 夹取速度
        'force': 300,    # 夹取力度
        'timeout': 3,    # 超时时间（秒）
    },
    'release': {
        'speed': 100,    # 释放速度
        'timeout': 3,    # 超时时间（秒）
    }
}

# 移动速度配置
MOVE_SPEED = 10  # 机械臂移动速度

# 最大尝试次数
MAX_ATTEMPTS = 5  # 夹取/释放操作的最大尝试次数 

# 机械臂配置
ROBOT1_CONFIG = {
    "ip": "192.168.3.18",
    "port": 8080,
    "initial_pose": [-0.04844, -0.269769, -0.101888, 3.109, -0.094, -1.592]
}

ROBOT2_CONFIG = {
    "ip": "192.168.3.19",
    "port": 8080,
    "initial_pose": [-0.053437, 0.24741, -0.120801, 3.114, -0.032, -2.935]
}

# 机械臂移动参数
MOVE_CONFIG = {
    "velocity": 10,
    "radius": 0,
    "connect": 0,
    "block": 1
} 