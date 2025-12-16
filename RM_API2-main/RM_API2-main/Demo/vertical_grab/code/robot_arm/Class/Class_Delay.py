import serial
import time

class RelayController:
    def __init__(self, port='/dev/ttyUSB2', baudrate=38400, timeout=1):
        """初始化继电器控制器
        Args:
            port (str): 串口号
            baudrate (int): 波特率
            timeout (int): 超时时间
        """
        self.ser = serial.Serial(port, baudrate=baudrate, timeout=timeout)

    def turn_on_relay_Y1(self):
        """打开继电器Y1"""
        command_on = b'\x01\x06\x00\x00\x00\x01\x48\x0A'
        self.ser.write(command_on)
        print("继电器 Y1 已打开")

    def turn_off_relay_Y1(self):
        """关闭继电器Y1"""
        command_off = b'\x01\x06\x00\x00\x00\x00\x89\xCA'
        self.ser.write(command_off)
        print("继电器 Y1 已关闭")

    def turn_on_relay_Y2(self):
        """打开继电器Y2"""
        command_on = b'\x01\x06\x00\x01\x00\x01\x19\xCA'
        self.ser.write(command_on)
        print("继电器 Y2 已打开")

    def turn_off_relay_Y2(self):
        """关闭继电器Y2"""
        command_off = b'\x01\x06\x00\x01\x00\x00\xD8\x0A'
        self.ser.write(command_off)
        print("继电器 Y2 已关闭")

    def close(self):
        """关闭串口连接"""
        self.ser.close()

if __name__ == "__main__":
    # 测试代码
    controller = RelayController()  # 创建控制器实例
    
    # 测试控制继电器 Y1 和 Y2
    controller.turn_on_relay_Y1()
    time.sleep(2)
    # controller.turn_off_relay_Y1()
    # time.sleep(2)

    controller.turn_on_relay_Y2()
    time.sleep(2)
    # controller.turn_off_relay_Y2()
    # time.sleep(2)

    # 关闭串口连接
    controller.close()
