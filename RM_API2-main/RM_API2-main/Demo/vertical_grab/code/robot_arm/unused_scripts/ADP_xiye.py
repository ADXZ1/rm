import serial

# 串口初始化函数
def init_serial(port='COM18', baudrate=115200, timeout=5):
    try:
        ser = serial.Serial(port, baudrate=baudrate, timeout=timeout)
        return ser
    except Exception as e:
        print("串口初始化失败:", e)
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

# 发送命令函数
def send_command(command_str, ser=None):
    try:
        if ser is None:  # 如果没有传入串口对象，则自行初始化（独立执行模式）
            ser = init_serial(port='COM4')
        if ser and ser.is_open:
            print("发送命令:", command_str)
            ser.write(command_str.encode('ascii'))
            response = ser.read(20)
            print("响应:", response.decode('ascii', errors='replace'))
        else:
            print("串口未打开")
    except Exception as e:
        print("串口通信出错:", e)

# 测试代码块
if __name__ == "__main__":
    ser = init_serial(port='COM4')  # 独立执行时初始化串口
    send_command(create_command('n', 800), ser)
