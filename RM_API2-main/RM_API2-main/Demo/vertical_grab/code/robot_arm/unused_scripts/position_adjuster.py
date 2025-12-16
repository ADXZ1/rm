import json
from config import POSITION_CONFIG_FILE

class PositionAdjuster:
    def __init__(self):
        # 从配置文件读取基准位置和当前偏移
        try:
            with open(POSITION_CONFIG_FILE, 'r', encoding='utf-8') as f:
                config = json.load(f)
                self.reference_position = config['reference_position']
                self.current_offset = config['current_offset']
        except Exception as e:
            print(f"读取位置配置文件失败: {str(e)}")
            # 使用默认值
            self.reference_position = {'x': 0, 'y': 0, 'angle': 0}
            self.current_offset = {'x': 0, 'y': 0, 'angle': 0}
        
    def get_adjusted_position(self, original_position):
        """
        根据当前偏移量调整机械臂位置
        :param original_position: 原始固定点位置 [x, y, z, rx, ry, rz]，米为单位
        :return: 调整后的位置，米为单位
        """
        # 复制原始位置以避免修改原始数据
        adjusted_position = list(original_position)
        
        # 计算转换量（转换量 = 偏移量 - 基准位置）
        x_transform = (self.current_offset['x'] - self.reference_position['x'])/100  # 单位转换
        y_transform = (self.current_offset['y'] - self.reference_position['y'])/100  # 单位转换
        
        # 调整位置 = 固定点位置 + 转换量
        adjusted_position[0] = original_position[0] + x_transform
        adjusted_position[1] = original_position[1] + y_transform
        
        return adjusted_position

# 测试代码
if __name__ == '__main__':
    # 创建位置调整器实例
    adjuster = PositionAdjuster()
    
    # 测试一个位置
    test_position = [-0.303379, 0.274441, -0.075986, -3.081, 0.137, -1.828]
    adjusted_pos = adjuster.get_adjusted_position(test_position)
    
    print("基准位置：")
    print(f"X: {adjuster.reference_position['x']:.3f} cm")
    print(f"Y: {adjuster.reference_position['y']:.3f} cm")
    
    print("\n当前偏移：")
    print(f"X: {adjuster.current_offset['x']:.3f} cm")
    print(f"Y: {adjuster.current_offset['y']:.3f} cm")
    
    print("\n转换量：")
    print(f"X: {(adjuster.current_offset['x'] - adjuster.reference_position['x']):.3f} cm")
    print(f"Y: {(adjuster.current_offset['y'] - adjuster.reference_position['y']):.3f} cm")
    
    print("\n位置调整测试：")
    print(f"原始位置: {test_position}")
    print(f"调整后的位置: {adjusted_pos}") 