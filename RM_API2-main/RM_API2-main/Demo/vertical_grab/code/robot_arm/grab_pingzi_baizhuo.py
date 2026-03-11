import os
import time
import cv2
import numpy as np
from vertical_grab.interface import vertical_catch
from Robot import RobotController  # 统一使用 RobotController 完成初始化
from config import *  # 所有参数从 config 配置

PICTURE_DIR = os.path.join(
    os.path.dirname(__file__),
    "..",
    "pictures"
)


def detect_target(image, yolo_model, sam_model, process_mask_fn, width=640, height=480, conf_thresh=0.7):
    """统一的视觉处理：YOLO -> SAM -> GMM，返回 mask、bbox、detected"""
    mask = np.zeros((height, width), dtype=np.uint8)
    detected = False
    bbox = None

    # 降低分辨率以提高推理速度 - 从1024x1024降低到416x416
    target_size = 416
    resized_image = cv2.resize(image, (target_size, target_size))

    # 优化YOLO推理参数
    yolo_results = yolo_model(
        resized_image,
        verbose=False,
        conf=conf_thresh,
        iou=0.45,  # IoU阈值
        max_det=1,  # 最多检测1个目标
        imgsz=(target_size, target_size)  # 明确指定输入尺寸
    )
    for result in yolo_results:
        for box in result.boxes:
            confidence = float(box.conf)
            if confidence < conf_thresh:
                continue

            detected = True
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()

            # 将bbox坐标缩放到原始图像尺寸
            scale_factor = width / target_size
            x1_orig = int(x1 * scale_factor)
            y1_orig = int(y1 * scale_factor)
            x2_orig = int(x2 * scale_factor)
            y2_orig = int(y2 * scale_factor)

            bbox = [x1_orig, y1_orig, x2_orig, y2_orig]

            sam_results = sam_model(image, bboxes=[bbox])
            if sam_results and len(sam_results) > 0:
                sam_mask = sam_results[0].masks.data[0].cpu().numpy()
                sam_mask = (sam_mask * 255).astype(np.uint8)
                improved_mask = process_mask_fn(image, sam_mask)
                mask = cv2.bitwise_or(mask, improved_mask)

            cv2.rectangle(image, (x1_orig, y1_orig), (x2_orig, y2_orig), (0, 255, 0), 2)

    return mask, bbox, detected


def execute_pick(robot, target_pose, drop_height):
    """动作原语：到上方 -> 下降 -> 夹取 -> 抬升"""
    above = target_pose.copy()
    ret = robot.rm_movej_p(above, v=MOVE_SPEED, r=0, connect=0, block=1)
    if ret != 0:
        raise Exception(f"移动到上方失败，错误码：{ret}")

    below = above.copy()
    below[2] -= drop_height
    ret = robot.rm_movel(below, v=MOVE_SPEED, r=0, connect=0, block=1)
    if ret != 0:
        raise Exception(f"下降到目标失败，错误码：{ret}")

    attempts = 0
    while attempts < MAX_ATTEMPTS:
        ret = robot.rm_set_gripper_pick_on(
            speed=GRIPPER_CONFIG['pick']['speed'],
            block=True,
            timeout=GRIPPER_CONFIG['pick']['timeout'],
            force=GRIPPER_CONFIG['pick']['force']
        )
        if ret == 0:
            break
        attempts += 1
        time.sleep(1)
    if attempts == MAX_ATTEMPTS:
        raise Exception("达到最大尝试次数，夹取仍未成功")

    ret = robot.rm_movel(above, v=MOVE_SPEED, r=0, connect=0, block=1)
    if ret != 0:
        raise Exception(f"抬升失败，错误码：{ret}")


def execute_place(robot, target_pose, drop_height):
    """动作原语：到上方 -> 下降 -> 松开 -> 抬升"""
    above = target_pose.copy()
    ret = robot.rm_movej_p(above, v=MOVE_SPEED, r=0, connect=0, block=1)
    if ret != 0:
        raise Exception(f"移动到放置上方失败，错误码：{ret}")

    below = above.copy()
    below[2] -= drop_height
    ret = robot.rm_movel(below, v=MOVE_SPEED, r=0, connect=0, block=1)
    if ret != 0:
        raise Exception(f"下降到放置高度失败，错误码：{ret}")

    attempts = 0
    while attempts < MAX_ATTEMPTS:
        ret = robot.rm_set_gripper_release(
            speed=GRIPPER_CONFIG['release']['speed'],
            block=True,
            timeout=GRIPPER_CONFIG['release']['timeout']
        )
        if ret == 0:
            break
        attempts += 1
        time.sleep(1)
    if attempts == MAX_ATTEMPTS:
        raise Exception("达到最大尝试次数，释放仍未成功")

    ret = robot.rm_movel(above, v=MOVE_SPEED, r=0, connect=0, block=1)
    if ret != 0:
        raise Exception(f"抬升失败，错误码：{ret}")


def place_at_fixed_position(robot):
    """使用动作原语完成放置"""
    execute_place(robot, PLACE_POSITION['above'].copy(), PLACE_POSITION['drop_height'])
    ret = robot.rm_movej_p(INITIAL_POSE, v=MOVE_SPEED, r=0, connect=0, block=1)
    if ret != 0:
        raise Exception(f"回到初始位姿失败，错误码：{ret}")


# def return_bottle_to_original(robot):
#     """把瓶子从放置点取回原位"""
#     execute_pick(robot, PLACE_POSITION['above'].copy(), PLACE_POSITION['drop_height'])
#     execute_place(robot, GRAB_POSITION['above'].copy(), GRAB_POSITION['drop_height'])
#     ret = robot.rm_movej_p(INITIAL_POSE, v=MOVE_SPEED, r=0, connect=0, block=1)
#     if ret != 0:
#         raise Exception(f"回到初始姿态失败，错误码：{ret}")


def capture_and_move(controller, robot, width=640, height=480):
    """获取一帧图像并执行抓取、放置"""
    try:
        controller.openclaw(robot)

        ret, initial_state = robot.rm_get_current_arm_state()
        if ret != 0:
            raise Exception(f"获取初始位姿失败，错误码：{ret}")
        initial_pose = initial_state['pose']

        color_image, depth_image, color_intr = controller.get_frames_from_gui()
        if color_image is None or depth_image is None or color_intr is None:
            raise Exception("无法获取图像或相机内参")

        os.makedirs(PICTURE_DIR, exist_ok=True)
        cv2.imwrite(os.path.join(PICTURE_DIR, 'original_image.jpg'),
                    cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR))

        mask, bbox, detected = detect_target(
            color_image,
            controller.yolo_model,
            controller.sam_model,
            controller.process_mask_with_gmm,
            width,
            height
        )
        if not detected:
            cv2.imwrite(os.path.join(PICTURE_DIR, 'failed_detection.jpg'),
                        cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR))
            raise Exception("未检测到目标")

        cv2.imwrite(os.path.join(PICTURE_DIR, 'mask_result.jpg'), mask)

        ret, state_dict = robot.rm_get_current_arm_state()
        if ret != 0:
            raise Exception(f"获取机械臂状态失败，错误码：{ret}")
        pose = state_dict['pose']

        above_object_pose, _, _ = vertical_catch(
            mask,
            depth_image,
            color_intr,
            pose,
            100,
            controller.gripper_offset,
            controller.rotation_matrix,
            controller.translation_vector
        )

        camera_above_pose = above_object_pose.copy()
        # camera_above_pose[0] -= 0.7
        ret = robot.rm_movej_p(camera_above_pose, v=15, r=0, connect=0, block=1)
        if ret != 0:
            raise Exception(f"移动到预备位置失败，错误码：{ret}")
        time.sleep(1)

        color_image, depth_image, color_intr = controller.get_frames_from_gui()
        if color_image is None or depth_image is None or color_intr is None:
            raise Exception("二次检测：无法获取图像或相机内参")

        mask, bbox, detected = detect_target(
            color_image,
            controller.yolo_model,
            controller.sam_model,
            controller.process_mask_with_gmm,
            width,
            height
        )
        if not detected:
            raise Exception("二次检测：未检测到目标")

        ret, current_state = robot.rm_get_current_arm_state()
        if ret != 0:
            raise Exception(f"获取当前状态失败，错误码：{ret}")
        current_pose = current_state['pose']

        _, _, final_pose = vertical_catch(
            mask,
            depth_image,
            color_intr,
            current_pose,
            100,
            controller.gripper_offset,
            controller.rotation_matrix,
            controller.translation_vector
        )

        final_pose[3] = controller.gripper_offset[0]
        final_pose[4] = controller.gripper_offset[1]
        final_pose[5] = controller.gripper_offset[2]

        above_target_pose = final_pose.copy()
        above_target_pose[2] = current_pose[2]
        above_target_pose[1] -= 0.015

        ret = robot.rm_movel(above_target_pose, v=15, r=0, connect=0, block=1)
        if ret != 0:
            raise Exception(f"移动到目标物体上方失败，错误码：{ret}")

        final_pose[1] -= 0.015
        final_pose[2] = -0.24
        ret = robot.rm_movel(final_pose, v=15, r=0, connect=0, block=1)
        if ret != 0:
            raise Exception(f"垂直下降失败，错误码：{ret}")

        execute_pick(robot, final_pose, drop_height=0)  # 已经在位，drop_height 0 表示只夹取

        ret = robot.rm_movej_p(initial_pose, v=15, r=0, connect=0, block=1)
        if ret != 0:
            raise Exception(f"回到初始位姿失败，错误码：{ret}")

        return True
    except Exception as exc:
        print(f"错误: {exc}")
        return False