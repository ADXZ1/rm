import cv2
from vertical_grab.interface import vertical_catch

depth_frame = cv2.imread("real_depth_frame.png", cv2.IMREAD_GRAYSCALE)
mask = cv2.imread("manual_catch_mask.png", cv2.IMREAD_GRAYSCALE)

color_intr = {"ppx": 326.721, "ppy": 252.721, "fx": 606.721, "fy": 607.55}
current_pose = [-0.06532000005245209, 0.004813000094145536, 0.3710620105266571, -3.058000087738037, 0.24199999868869781,
                0.041999999433755875]

arm_gripper_length = 0.02
vertical_rx_ry_rz = [3.14, 0, -0.020999999716877937]

rotation_matrix = [[0.97546, 0.20096, 0.089965], [-0.21137, 0.96912, 0.12699],
                   [-0.061666, -0.14289, 0.98782]]
translation_vector = [0.02655408960616487, -0.18542431314038166, 0.65143524836874]

result = ([-0.13396845376699643, -0.03954672752039725, 0.4727158838499269, 3.14, 0, -0.020999999716877937],
          [-0.13396845376699643, -0.03954672752039725, 0.4727158838499269, 3.14, 0, -0.020999999716877937],
          [-0.1339728013898331, -0.03975372674782365, 0.3427160487253468, 3.14, 0, -0.020999999716877937])


def test_vertical_catch():
    assert vertical_catch(mask, depth_frame, color_intr, current_pose, arm_gripper_length, vertical_rx_ry_rz,
                          rotation_matrix,
                          translation_vector, True) == result


if __name__ == "__main__":
    test_vertical_catch()
