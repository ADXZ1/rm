import time
from Robotic_Arm.rm_robot_interface import *
from Class.Class_Kuaihuanshou import Kuaihuanshou
from Class.Class_Delay import RelayController

relay_controller = RelayController(port='/dev/ttyUSB2')
khs = Kuaihuanshou(port='COM4')
robot = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
handle = robot.rm_create_robot_arm("192.168.2.18", 8080)

# 点位
points = [
    {"point_name": "huandian1", "pose": [-374.88 / 1000, 325.69 / 1000, -70 / 1000, -3.141, -0.0, -0.278]},  
    {"point_name": "huandian2", "pose": [-135.95 / 1000, 324.0 / 1000, -70 / 1000, -3.141, -0.0, -0.278]},  
    {"point_name": "huandian3", "pose": [-136.08 / 1000, 324.0 / 1000, -95.15 / 1000, -3.141, -0.00, -0.278]},   
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
        ret = robot.rm_movej_p(points[0]["pose"], v=10, r=0, connect=0, block=1)
        ret = robot.rm_movel(points[1]["pose"], v=10, r=0, connect=0, block=1)
        ret = robot.rm_movel(points[2]["pose"], v=10, r=0, connect=0, block=1)

        # 执行解锁操作
        print("执行解锁操作")
        res = khs.send_command('open')
        time.sleep(0.5)
        status = khs.send_command('status')
        if status == "locked":
            res = khs.send_command('open')

        # 开启继电器 Y2
        relay_ctrl.turn_on_relay_Y2()  # 开启继电器
        time.sleep(2)    # 等待 2 秒

        # 继续执行机械臂运动任务
        ret = robot.rm_movel(points[1]["pose"], v=10, r=0, connect=0, block=1)
        ret = robot.rm_movel(points[0]["pose"], v=10, r=0, connect=0, block=1)

        print("任务执行完成")
    finally:
        # 确保关闭连接
        relay_ctrl.close()
        khs.close()

if __name__ == "__main__":
    execute_robot_task_unlock(robot, points, relay_controller, khs)
