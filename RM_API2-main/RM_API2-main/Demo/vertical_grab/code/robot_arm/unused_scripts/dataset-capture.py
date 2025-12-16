import cv2
import pyrealsense2 as rs
import numpy as np
import os
from datetime import datetime

def capture_dataset(save_dir="dataset", width=640, height=480, fps=60):
    """
    打开RealSense相机并采集数据集
    按 's' 保存当前帧
    按 'q' 退出程序
    
    Args:
        save_dir: 保存图片的目录
        width: 图像宽度
        height: 图像高度
        fps: 帧率
    """
    # 创建保存目录
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)
        print(f"创建保存目录: {save_dir}")
    
    # 初始化RealSense
    pipeline = rs.pipeline()
    config = rs.config()
    
    # 只启用彩色相机
    config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
    
    try:
        pipeline.start(config)
        print("\n==== 相机已启动 ====")
        print("按 's' 保存图片")
        print("按 'q' 退出程序")
        
        count = 0  # 计数器
        
        while True:
            # 等待一帧数据
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            
            if not color_frame:
                continue
            
            # 转换为numpy数组
            color_image = np.asanyarray(color_frame.get_data())
            
            # 显示图像
            cv2.imshow('RealSense', color_image)
            
            # 等待按键
            key = cv2.waitKey(1)
            
            # 按's'保存图片
            if key == ord('s'):
                count += 1
                # 生成时间戳
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                # 保存彩色图
                filename = os.path.join(save_dir, f"{timestamp}_{count}.jpg")
                cv2.imwrite(filename, color_image)
                print(f"保存图片 {count}: {filename}")
            
            # 按'q'退出
            elif key == ord('q'):
                print("\n==== 程序结束 ====")
                print(f"共保存 {count} 张图片")
                break
    
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__": 
    try:
        capture_dataset()
    except Exception as e:
        print(f"错误: {str(e)}")