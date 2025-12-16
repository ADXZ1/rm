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

# 放置位置相关
PLACE_POSITION = {
    'above': [0.042, 0.1525, -0.2083, 3.141, 0.00, -1.603],  # 离心管架放置点上方位置
    'drop_height': 0.07,  # 从离心管架上方准备向下把瓶子抓起来时的下降高度

    'table' : [-151.986 / 1000 , 313.002 / 1000 , -215 / 1000 , 3.143 , 0 , -1.655]
}
# 抓取位置相关
GRAB_POSITION = {
    'above': [-0.27709961265851596, 0.3721638427921397, -0.158696, 3.142, 0, 3.128],  # 试管预备放置位置的上方位置
    'drop_height': 0.08,  # 放置时的下降高度
}

# 抓取位置相关
GRAB_POSITION_reput = {
    'above': [-0.156, 0.372, -0.184, 3.142, 0, -1.68],  # 试管预备放置位置的上方位置
    'drop_height': 0.055,  # 放置时的下降高度
}

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
    "ip": "192.168.2.18",
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

# 吸液位置
XIYE_CONFIG = {
    "above": [92.278 / 1000, -164.573 / 1000, 0.687 / 1000 , -3.141, 0.0, 2.928],  # 离心管架放置点上方位置
    "down": [92.278 / 1000, -164.573 / 1000, -102.676 / 1000 , -3.141, 0.0, 2.902],  # 吸液位置
    "down_height": 0.02,  # 吸液位置下降高度
}


# 吐液位置
TUYE_CONFIG = {
    "st": [-176 / 1000, -185 / 1000, 256 / 1000, -2.9, 0.274, 2.707],  # 中间位置
    "pos1" : [-359.454 / 1000, -300.000 / 1000, 138.013 / 1000, 3.141 , 0.147 , 1.231],  # 第一个吐液位置
    "pos2": [-201.925 / 1000, -318.48 / 1000, 141.737 / 1000, 3.14, 0.078, 1.196],  # 第二个吐液位置
    "pos3": [-93.004 / 1000, -336.992 / 1000, 142.009 / 1000, 3.14, 0.089, 1.728],  # 第三个吐液位置
    "pos4": [13.784 / 1000, -306.642 / 1000, 143.713 / 1000, 3.041, 0.183, 1.673],  # 第四个吐液位置
}

INIT_CONFIG = {
    "init_pos" : [-63.551 / 1000 , -141.901 / 1000 , 183.018 / 1000 , -3.141 , 0.0 , 3.027],
    "init_pos1" : [-105 / 1000 , 99 / 1000 , 123 / 1000 , 3.141 , 0 , -1.6]
}