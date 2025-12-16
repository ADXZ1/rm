import serial
import time

# 串口初始化函数
def init_serial(port='COM18', baudrate=115200, timeout=5, max_retries=3):
    for attempt in range(max_retries):
        try:
            # 如果串口已经被占用，先尝试关闭
            try:
                temp_ser = serial.Serial(port)
                temp_ser.close()
                time.sleep(1)  # 等待串口释放
            except:
                pass
            
            ser = serial.Serial(port, baudrate=baudrate, timeout=timeout)
            print(f"串口 {port} 初始化成功")
            return ser
        except Exception as e:
            print(f"第 {attempt + 1} 次尝试初始化串口失败: {str(e)}")
            time.sleep(1)  # 等待一秒后重试
    
    print(f"串口 {port} 初始化失败，已重试 {max_retries} 次")
    return None

# CRC 校验函数 (Modbus CRC-16)
def cal_crc(byte_arr):
    itemp = 0xFFFF
    for b in byte_arr:
        itemp ^= b
        for _ in range(8):
            if itemp & 0x0001:
                itemp = (itemp >> 1) ^ 0xA001
            else:
                itemp >>= 1
    return itemp

# 十进制转十六进制
def decimal_to_hex(decimal_number):
    return f"{decimal_number:04X}"

# 创建命令帧
def create_command(func_code, volume=None):
    addr = "01"
    data = decimal_to_hex(volume) if volume is not None else ""
    frame = f">{addr}{func_code}{data}"
    crc_value = cal_crc(frame.encode('ascii'))
    crc_str = f"{crc_value:04X}"
    return frame + crc_str

def tuye_all(ser=None):
    """
    一次性吐出所有液体的函数
    """
    command_str = ">01p000061AC"  # 完全吐液的固定指令
    try:
        if ser is None:  # 如果没有传入串口对象，则自行初始化
            ser = init_serial()
            if ser is None:
                print("无法执行吐液操作：串口初始化失败")
                return False
        
        if ser and ser.is_open:
            print("发送完全吐液命令:", command_str)
            ser.write(command_str.encode('ascii'))
            response = ser.read(20)
            print("响应:", response.decode('ascii', errors='replace'))
            return True
        else:
            print("串口未打开")
            return False
    except Exception as e:
        print("串口通信出错:", e)
        return False

# 发送命令函数
def send_command(command_str, ser=None):
    try:
        if ser is None:  # 如果没有传入串口对象，则自行初始化
            ser = init_serial()
            if ser is None:
                print("无法执行命令：串口初始化失败")
                return False
        
        if ser and ser.is_open:
            print("发送命令:", command_str)
            ser.write(command_str.encode('ascii'))
            response = ser.read(20)
            print("响应:", response.decode('ascii', errors='replace'))
            return True
        else:
            print("串口未打开")
            return False
    except Exception as e:
        print("串口通信出错:", e)
        return False

# 测试代码块
if __name__ == "__main__":
    try:
        print("开始测试吐液功能...")
        ser = init_serial(port='COM4')
        # send_command(create_command("M"),ser)  # 吐出 1000ml 的液体
        # send_command(create_command("p", 20),ser)  # 吐出 1000ml 的液体
        if ser:
            if tuye_all(ser):  # 直接使用已初始化的串口对象
                print("吐液操作执行成功")
            else:
                print("吐液操作执行失败")
    except Exception as e:
        print(f"测试过程中出现错误: {str(e)}")
    finally:
        print("测试完成")

