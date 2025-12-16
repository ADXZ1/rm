import time
from Robotic_Arm.rm_robot_interface import *
from Class.Class_Kuaihuanshou import Kuaihuanshou
from Class.Class_Delay import RelayController
relay_controller = RelayController(port='/dev/ttyUSB2')
khs = Kuaihuanshou(port='/dev/ttyUSB3')
robot = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
handle = robot.rm_create_robot_arm("192.168.3.19", 8080)

# 点位
points = [
    {"point_name": "huandian1", "pose": [-527.646 / 1000, -181.554 / 1000, -131.364 / 1000, -3.141, -0.0, -1.954]},  
    {"point_name": "huandian2", "pose": [-500.344 / 1000, 305.554 / 1000, -183.364 / 1000, -3.141, -0.0, -2.354]},  
    {"point_name": "huandian3", "pose": [-410.364 / 1000, 368.539 / 1000, -195.885 / 1000, -3.141, -0.0, -0.687]}, 
    {"point_name": "huandian4", "pose": [-262.394 / 1000, 431.594 / 1000, -159.673 / 1000, -3.141, -0.000, -0.784]},  
    {"point_name": "huandian5", "pose": [-95.722 / 1000, 404.732 / 1000, -159.673 / 1000, -3.124, -0.024, -0.792]},  
    {"point_name": "huandian5", "pose": [-95.722 / 1000, 404.732 / 1000, -279.294 / 1000, -3.124, -0.024, -0.792]},    
    ]
ret, state = robot.rm_get_current_arm_state()
if ret != 0:
    raise Exception(f"获取机械臂状态失败，错误码：{ret}")
if state.get('error_code', 0) != 0:
    raise Exception(f"机械臂存在错误，错误码：{state['error_code']}")
print(f"机械臂当前状态: {state}")

def execute_robot_task_unlock(robot, points, relay_ctrl, khs):
    """
    执行机械臂任务，包括移动到指定点和控制快换手操作。

    :param robot: 机械臂对象
    :param points: 点位列表，包含每个点的位置
    :param relay_ctrl: 继电器控制器对象
    :param khs: 快换手控制对象
    """
    try:
        # 执行机械臂的运动任务
        print("移动到初始位置...")
        ret = robot.rm_movej_p(points[0]["pose"], v=10, r=0, connect=0, block=1)
        if ret != 0:
            raise Exception(f"移动失败，错误码: {ret}")
        
        ret = robot.rm_movej_p(points[1]["pose"], v=10, r=0, connect=0, block=1)
        if ret != 0:
            raise Exception(f"移动失败，错误码: {ret}")
        ret = robot.rm_movej_p(points[2]["pose"], v=10, r=0, connect=0, block=1)
        ret = robot.rm_movej_p(points[3]["pose"], v=10, r=0, connect=0, block=1)
        ret = robot.rm_movej_p(points[4]["pose"], v=10, r=0, connect=0, block=1)
        ret = robot.rm_movel(points[5]["pose"], v=5, r=0, connect=0, block=1)

        # # 关闭继电器 Y2
        # relay_ctrl.turn_off_relay_Y2()  # 关闭继电器
        # time.sleep(2)    # 等待 2 秒
        
        # # 循环尝试上锁直到成功
        # max_attempts = 5  # 最大尝试次数
        # attempt = 0
        # while True:
        #     print(f"第 {attempt + 1} 次尝试上锁...")
            
        #     # 执行上锁操作
        #     res = khs.send_command('close')
        #     time.sleep(0.5)  # 等待状态更新
            
        #     # 检查状态
        #     status = khs.send_command('status')
        #     print(f"当前状态: {status}")
            
        #     if status == "locked":
        #         print("上锁成功！")
        #         break
            
        #     attempt += 1
        #     if attempt >= max_attempts:
        #         print(f"警告：尝试{max_attempts}次后仍未成功上锁")
        #         break
                
        #     print("上锁未成功，等待后重试...")
        #     time.sleep(1)  # 重试前等待
        
        # # 继续执行机械臂运动任务
        ret = robot.rm_movel(points[4]["pose"], v=10, r=0, connect=0, block=1)
        ret = robot.rm_movel(points[3]["pose"], v=10, r=0, connect=0, block=1)
        ret = robot.rm_movej_p(points[2]["pose"], v=10, r=0, connect=0, block=1)
        if ret != 0:
            raise Exception(f"移动失败，错误码: {ret}")
        
        ret = robot.rm_movej_p(points[1]["pose"], v=10, r=0, connect=0, block=1)
        if ret != 0:
            raise Exception(f"移动失败，错误码: {ret}")
        ret = robot.rm_movej_p(points[0]["pose"], v=10, r=0, connect=0, block=1)

        print("任务执行完成")
    finally:
        # 确保关闭连接
        relay_ctrl.close()
        khs.close()

if __name__ == "__main__":
    execute_robot_task_unlock(robot, points, relay_controller, khs)