import serial
import struct

def crc16(data):
    crc = 0xFFFF
    for byte in data:
        if isinstance(byte, str):  
            byte = ord(byte)  
        
        crc ^= byte
        for _ in range(8):
            if (crc & 0x0001) != 0:
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1
    return crc

def create_command(switch_command):
    crc = crc16(switch_command)
    crc_bytes = struct.pack('<H', crc)
    command_packet = switch_command + crc_bytes
    return command_packet

def send_command(serial_port, switch_command):
    command = create_command(switch_command)
    serial_port.write(command)
    response = serial_port.read(size=len(command)) 
    return response

def connect(port, baudrate):
    return serial.Serial(port, baudrate, timeout=3)  

def control_command(ser, comm):
    # Define switch commands in a dictionary
    switch_commands = {
        'close': bytes([0x53, 0x26, 0x01, 0x01, 0x01]),
        'open': bytes([0x53, 0x26, 0x01, 0x01, 0x02]),
        'status': bytes([0x53, 0x26, 0x02, 0x01, 0x01]),
        'temp': bytes([0x53, 0x26, 0x03, 0x01, 0x01]),
        'power_on': bytes([0x53, 0x26, 0x04, 0x01, 0x01]),
        'power_off': bytes([0x53, 0x26, 0x04, 0x01, 0x02]),
        'power_status': bytes([0x53, 0x26, 0x05, 0x01, 0x01])
    }

    try:
        response = send_command(ser, switch_commands[comm])
        
        # 添加响应检查
        if not response or len(response) < 5:
            print(f"警告: 接收到的响应数据不完整 - 长度: {len(response) if response else 0}")
            return "error"

        # 打印调试信息
        print(f"收到响应: {[hex(b) for b in response]}")

        if comm == 'temp':
            return int(response[4])
        elif comm in ['close', 'open', 'power_on', 'power_off']:
            if response[:5] == switch_commands[comm]:
                return True
            else:
                return False
        elif comm == 'status':  # 处理状态
            if response[4] == 1:
                return "locked"
            elif response[4] == 2:
                return "unlocked"
            else:
                return "unknown"
        else:
            if response[4] == 1:
                return "on"
            elif response[4] == 2:
                return "off"
            else:
                return "error"
    except Exception as e:
        print(f"控制命令执行出错: {str(e)}")
        return "error"
    finally:
        print(f"命令 {comm} 执行完成")

if __name__ == "__main__":
    ser = connect('/dev/ttyCH341USB1', 115200)  
    res = control_command(ser, 'open')
    print("response", res)