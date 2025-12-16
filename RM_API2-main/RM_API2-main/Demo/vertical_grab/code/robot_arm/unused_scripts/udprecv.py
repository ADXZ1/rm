import socket
import json
import threading

class UDPReceiver:
    def __init__(self, host='192.168.1.161', port=22222):
        """
        初始化接收端。
        :param host: 本机 IP 地址
        :param port: 监听的端口号
        """
        self.host = host
        self.port = port
        self.buffer_size = 1024
        self.is_running = False
        self.socket = None
        self.data_callback = None  # 回调函数，用于处理接收到的数据

    def start(self):
        """
        启动接收端，开始监听 UDP 数据。
        """
        if self.is_running:
            print("Receiver is already running.")
            return

        self.is_running = True
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind((self.host, self.port))
        print(f"UDP receiver started on {self.host}:{self.port}")

        # 启动一个线程来接收数据
        threading.Thread(target=self._receive_loop, daemon=True).start()

    def stop(self):
        """
        停止接收端。
        """
        if not self.is_running:
            print("Receiver is not running.")
            return

        self.is_running = False
        if self.socket:
            self.socket.close()
        print("UDP receiver stopped.")

    def set_data_callback(self, callback):
        """
        设置回调函数，用于处理接收到的数据。
        :param callback: 回调函数，接收一个字典作为参数
        """
        self.data_callback = callback

    def _receive_loop(self):
        """
        内部方法：循环接收 UDP 数据。
        """
        while self.is_running:
            try:
                data, addr = self.socket.recvfrom(self.buffer_size)  # 接收数据
                message = data.decode('utf-8')  # 解码为字符串
                print(f"Received message from {addr}: {message}")

                # 尝试解析 JSON 数据
                try:
                    parsed_data = json.loads(message)
                    print(f"Parsed data: {parsed_data}")

                    # 如果设置了回调函数，则调用回调函数
                    if self.data_callback:
                        self.data_callback(parsed_data)
                except json.JSONDecodeError as e:
                    print(f"Failed to parse JSON: {e}")
            except socket.error as e:
                if self.is_running:
                    print(f"Socket error: {e}")
                break

# 示例回调函数
def process_received_data(data):
    """
    处理接收到的数据。
    :param data: 接收到的 JSON 数据（字典格式）
    """
    print("Processing received data:")
    print(f"ID: {data.get('id')}, X: {data.get('x')}, Y: {data.get('y')}, Angle: {data.get('angle')}")

# 测试接收端
if __name__ == '__main__':
    # 创建接收端实例
    receiver = UDPReceiver(host='192.168.1.161', port=22222)

    # 设置回调函数
    receiver.set_data_callback(process_received_data)

    # 启动接收端
    receiver.start()

    try:
        # 主线程保持运行，等待接收数据
        while True:
            pass
    except KeyboardInterrupt:
        # 捕获 Ctrl+C，优雅地停止接收端
        receiver.stop()