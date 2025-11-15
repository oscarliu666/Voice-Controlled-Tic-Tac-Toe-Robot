"""
机械臂控制模块
负责机械臂的移动、夹爪控制、pick & place 操作
"""

import numpy as np
import mujoco

from so101_mujoco_utils import (
    move_to_pose_cubic,
    hold_position,
    convert_to_dictionary,
    send_position_command,
)
from so101_inverse_kinematics import get_inverse_kinematics

from cube_config import (
    OPEN_GRIPPER, CLOSED_GRIPPER,
    BOARD_POSITIONS, INITIAL_ARM_CONFIG,
    PILE_XY, PILE_BASE_Z, BLOCK_HEIGHT,
    INITIAL_PILE_COUNTS
)


# 定义末端执行器的旋转矩阵（保持夹爪朝下，且夹爪方向对齐 X-Y 轴）
def get_gripper_orientation():
    """
    返回夹爪的标准朝向旋转矩阵
    让夹爪始终从上方垂直抓取，且夹爪开合方向对齐方块的边
    """
    # 这个旋转矩阵让夹爪垂直朝下，夹取方向沿着 Y 轴
    return np.eye(3)  


def move_gripper_only(m, d, viewer, target_angle, duration=0.5):
    """
    只控制 gripper，从当前角度平滑插值到 target_angle，
    其余关节保持调用时的姿态不变
    """
    start_time = d.time
    start_dict = convert_to_dictionary(d.qpos)
    start_angle = start_dict["gripper"]

    while True:
        t = d.time - start_time
        if t > duration:
            break

        alpha = min(t / duration, 1.0)
        cmd = dict(start_dict)
        cmd["gripper"] = (1.0 - alpha) * start_angle + alpha * target_angle

        send_position_command(d, cmd)
        mujoco.mj_step(m, d)
        viewer.sync()


def pick_and_place_cube(
    m, d, viewer, cube_manager, cube, cell_name,
    hover_height=0.05, move_duration=1.0
):
    """
    用 SO-101 把指定 cube 从堆叠位置拿起来，放到棋盘格子
    """
    if cell_name not in BOARD_POSITIONS:
        print(f"[WARN] 格子 {cell_name} 不在 BOARD_POSITIONS 里")
        return

    # 堆叠位置
    stack_pos = cube["pos"].copy()
    stack_pos[0] -= 0.005           # 往负 x 方向挪 0.5cm
    stack_pos_above = stack_pos.copy()
    stack_pos_above[2] += hover_height

    # 棋盘位置
    board_pos = np.array(BOARD_POSITIONS[cell_name], dtype=float)
    board_pos_above = board_pos.copy()
    board_pos_above[2] += hover_height

    current_dict = convert_to_dictionary(d.qpos)

    # 1. 去堆叠上方（张开）
    config_stack_above = get_inverse_kinematics(stack_pos_above, np.eye(3))
    config_stack_above["gripper"] = OPEN_GRIPPER
    move_to_pose_cubic(m, d, viewer, current_dict, config_stack_above, move_duration)

    # 2. 下到方块中心（保持张开）
    config_stack_down = get_inverse_kinematics(stack_pos, np.eye(3))
    config_stack_down["gripper"] = OPEN_GRIPPER
    move_to_pose_cubic(m, d, viewer, config_stack_above, config_stack_down, move_duration / 2.0)

    # 3. 闭合夹爪
    move_gripper_only(m, d, viewer, CLOSED_GRIPPER, 0.6)

    # 4. 提回堆叠上方（保持闭合）
    stack_above_closed = convert_to_dictionary(d.qpos)
    move_to_pose_cubic(m, d, viewer, stack_above_closed, stack_above_closed, move_duration / 4.0)

    # 5. 堆叠上方 → 棋盘上方（闭合）
    config_board_above = get_inverse_kinematics(board_pos_above, np.eye(3))
    config_board_above["gripper"] = CLOSED_GRIPPER
    move_to_pose_cubic(m, d, viewer, stack_above_closed, config_board_above, move_duration)

    # 6. 下到棋盘格子（闭合）
    config_board_down = get_inverse_kinematics(board_pos, np.eye(3))
    config_board_down["gripper"] = CLOSED_GRIPPER
    move_to_pose_cubic(m, d, viewer, config_board_above, config_board_down, move_duration / 2.0)

    # 7. 张开夹爪（放下）
    move_gripper_only(m, d, viewer, OPEN_GRIPPER, 0.4)

    # 更新棋子位置和状态
    cube_manager.move_cube_to(viewer, cube, board_pos)
    cube["available"] = False

    # 8. 抬回棋盘上方
    config_board_above_open = get_inverse_kinematics(board_pos_above, np.eye(3))
    config_board_above_open["gripper"] = OPEN_GRIPPER
    move_to_pose_cubic(m, d, viewer, convert_to_dictionary(d.qpos), 
                      config_board_above_open, move_duration / 2.0)

    hold_position(m, d, viewer, 0.2)


def return_cube_to_pile(
    m, d, viewer, cube_manager, cube, target_pos,
    hover_height=0.05, move_duration=1.0
):
    """
    把已经在棋盘上的 cube 再次 pick & place 回某个堆叠位置
    """
    board_pos = cube["pos"].copy()
    board_pos[0] -= 0.005           # pick 棋盘上的方块时也往负 x 偏 1cm
    board_pos_above = board_pos.copy()
    board_pos_above[2] += hover_height

    pile_pos = np.array(target_pos, dtype=float)
    pile_pos_above = pile_pos.copy()
    pile_pos_above[2] += hover_height

    current_dict = convert_to_dictionary(d.qpos)

    # 1. 去棋盘上方（张开）
    config_board_above = get_inverse_kinematics(board_pos_above, np.eye(3))
    config_board_above["gripper"] = OPEN_GRIPPER
    move_to_pose_cubic(m, d, viewer, current_dict, config_board_above, move_duration)

    # 2. 下到棋盘方块中心（张开）
    config_board_down = get_inverse_kinematics(board_pos, np.eye(3))
    config_board_down["gripper"] = OPEN_GRIPPER
    move_to_pose_cubic(m, d, viewer, config_board_above, config_board_down, move_duration / 2.0)

    # 3. 闭合夹爪
    move_gripper_only(m, d, viewer, CLOSED_GRIPPER, 0.4)
    cube_manager.hide_cube(viewer, cube)

    # 4. 提回棋盘上方（保持闭合）
    board_above_closed = convert_to_dictionary(d.qpos)
    move_to_pose_cubic(m, d, viewer, board_above_closed, board_above_closed, move_duration / 2.0)

    # 5. 棋盘上方 → 堆叠上方（闭合）
    config_pile_above = get_inverse_kinematics(pile_pos_above, np.eye(3))
    config_pile_above["gripper"] = CLOSED_GRIPPER
    move_to_pose_cubic(m, d, viewer, board_above_closed, config_pile_above, move_duration)

    # 6. 下到堆叠位置（闭合）
    config_pile_down = get_inverse_kinematics(pile_pos, np.eye(3))
    config_pile_down["gripper"] = CLOSED_GRIPPER
    move_to_pose_cubic(m, d, viewer, config_pile_above, config_pile_down, move_duration / 2.0)

    # 7. 张开夹爪（放下）
    move_gripper_only(m, d, viewer, OPEN_GRIPPER, 0.4)

    # 更新棋子位置和状态
    cube_manager.move_cube_to(viewer, cube, pile_pos)
    cube["available"] = True

    # 8. 抬回堆叠上方
    config_pile_above_open = get_inverse_kinematics(pile_pos_above, np.eye(3))
    config_pile_above_open["gripper"] = OPEN_GRIPPER
    move_to_pose_cubic(m, d, viewer, convert_to_dictionary(d.qpos),
                      config_pile_above_open, move_duration / 2.0)

    hold_position(m, d, viewer, 0.2)


def reset_all_cubes(m, d, viewer, cube_manager):
    """
    游戏结束后：让机械臂把所有在棋盘上的棋子送回各自的堆
    """
    print("开始复原所有棋子到初始堆叠位置...")
    hold_position(m, d, viewer, 0.2)

    pile_order = ["X_main", "X_side", "O_main", "O_side"]

    for pile_name in pile_order:
        xy = PILE_XY[pile_name]

        # 这个堆的所有 cube
        pile_cubes = [c for c in cube_manager.cubes if c["pile"] == pile_name]
        
        # 留在堆里的和在棋盘上的
        available_cubes = [c for c in pile_cubes if c["available"]]
        board_cubes = [c for c in pile_cubes if not c["available"]]

        if not board_cubes:
            continue

        current_layers = len(available_cubes)

        # 逐个送回堆顶
        for i, cube in enumerate(board_cubes):
            level = current_layers + i
            target_pos = np.array(
                [xy[0], xy[1], PILE_BASE_Z + level * BLOCK_HEIGHT],
                dtype=float,
            )
            return_cube_to_pile(m, d, viewer, cube_manager, cube, target_pos)

    print("所有棋子已复原到初始堆叠。")

    # 回到初始姿态
    current_dict = convert_to_dictionary(d.qpos)
    move_to_pose_cubic(m, d, viewer, current_dict, INITIAL_ARM_CONFIG, 2.0)
    hold_position(m, d, viewer, 0.5)