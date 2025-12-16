import time
from Robotic_Arm.rm_robot_interface import *
from Class.Class_Kuaihuanshou import Kuaihuanshou
from Class.Class_Delay import RelayController

robot = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
handle = robot.rm_create_robot_arm("192.168.2.18", 8080)

# 初始化控制器
relay_controller = RelayController(port='/dev/ttyUSB2')
khs = Kuaihuanshou(port='COM4')

# 点位
points = [
    {"point_name": "huandian1", "pose": [-417.736 / 1000,  233.564/ 1000, -61.276 / 1000, -3.141, -0.001, -0.277]},  
    {"point_name": "huandian2", "pose": [-135.95 / 1000,  233.564/ 1000, -61.276 / 1000, -3.141, -0.001, -0.277]},   
    {"point_name": "huandian3", "pose": [-136.5 / 1000, 233.564 / 1000, -96.448 / 1000, -3.141, -0.025, -0.277]},
]

ret, state = robot.rm_get_current_arm_state()
if ret != 0:
    raise Exception(f"获取机械臂状态失败，错误码：{ret}")
if state.get('error_code', 0) != 0:
    raise Exception(f"机械臂存在错误，错误码：{state['error_code']}")
print(f"机械臂当前状态: {state}")

def execute_robot_task_unlock(robot, points, relay_controller, khs):
    """
    执行机械臂任务，包括移动到指定点和控制继电器操作。

    :param robot: 机械臂对象
    :param points: 点位列表，包含每个点的位置
    :param relay_controller: 继电器控制器对象
    :param khs: 快换手控制器对象
    """
    try:
        # 执行机械臂的运动任务
        print("开始移动到初始位置...")
        ret = robot.rm_movej_p(points[0]["pose"], v=10, r=0, connect=0, block=1)
        ret = robot.rm_movel(points[1]["pose"], v=10, r=0, connect=0, block=1)
        ret = robot.rm_movel(points[2]["pose"], v=5, r=0, connect=0, block=1)
        print("到达目标位置")

        # 关闭继电器 Y1
        print("准备关闭继电器Y1...")
        relay_controller.turn_off_relay_Y1()
        print("继电器Y1已关闭")
        time.sleep(2)

        # 循环尝试上锁直到成功
        max_attempts = 5  # 最大尝试次数
        attempt = 0
        while True:
            print(f"第 {attempt + 1} 次尝试上锁...")
            
            # 执行上锁操作
            res = khs.send_command('close')
            time.sleep(0.5)  # 等待状态更新
            
            # 检查状态
            status = khs.send_command('status')
            print(f"当前状态: {status}")
            
            if status == "locked":
                print("上锁成功！")
                break
            
            attempt += 1
            if attempt >= max_attempts:
                print(f"警告：尝试{max_attempts}次后仍未成功上锁")
                break
                
            print("上锁未成功，等待后重试...")
            time.sleep(1)  # 重试前等待

        # 继续执行机械臂运动任务
        print("开始返回动作...")
        ret = robot.rm_movel(points[1]["pose"], v=10, r=0, connect=0, block=1)
        ret = robot.rm_movel(points[0]["pose"], v=10, r=0, connect=0, block=1)

    except Exception as e:
        print(f"执行过程中出现错误: {str(e)}")
        raise
    finally:
        print("任务执行完成")

if __name__ == "__main__":
    execute_robot_task_unlock(robot, points, relay_controller, khs)