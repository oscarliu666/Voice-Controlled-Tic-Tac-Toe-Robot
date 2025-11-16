import mujoco
import time 
import numpy as np

def convert_to_dictionary(qpos):
    return {
        'shoulder_pan':  qpos[0] * 180.0 / 3.14159,   
        'shoulder_lift': qpos[1] * 180.0 / 3.14159,
        'elbow_flex':    qpos[2] * 180.0 / 3.14159,
        'wrist_flex':    qpos[3] * 180.0 / 3.14159,
        'wrist_roll':    qpos[4] * 180.0 / 3.14159,
        'gripper':       qpos[5] * 100.0 / 3.14159    
    }

def convert_to_list(dictionary):
    return [
        dictionary['shoulder_pan']  * 3.14159 / 180.0,
        dictionary['shoulder_lift'] * 3.14159 / 180.0,
        dictionary['elbow_flex']    * 3.14159 / 180.0,
        dictionary['wrist_flex']    * 3.14159 / 180.0,
        dictionary['wrist_roll']    * 3.14159 / 180.0,
        dictionary['gripper']       * 3.14159 / 100.0
    ]

def set_initial_pose(d, position_dict):
    pos = convert_to_list(position_dict)
    d.qpos[:] = pos

# def send_position_command(d, position_dict):
#     pos = convert_to_list(position_dict)
#     d.ctrl[:] = pos

def send_position_command(d, position_dict, velocity_dict=None):
    pos = convert_to_list(position_dict)
    vel = convert_to_list(velocity_dict) if velocity_dict is not None else [0.0]*6
    d.ctrl = np.hstack([pos, vel])


def move_to_pose_cubic(m, d, viewer, starting_position, desired_position, duration):
    """
    用三次多项式 (cubic) 做关节空间轨迹规划，
    只对 5 个大关节做插值，gripper 保持当前值不动。
    """
    t0 = d.time
    done = False

    # 需要插值的关节（不包括 gripper）
    joints_to_interpolate = [
        "shoulder_pan",
        "shoulder_lift",
        "elbow_flex",
        "wrist_flex",
        "wrist_roll",
    ]

    while not done:
        t = d.time - t0
        if t >= duration:
            done = True

        position_dict = {}
        velocity_dict = {}

        # 1) 对五个大关节做 cubic 插值
        for joint in joints_to_interpolate:
            pos, vel = cubic_interpolation(t, joint, starting_position, desired_position, duration)
            position_dict[joint] = pos
            velocity_dict[joint] = vel

        # 2) gripper：保持当前角度、不做轨迹
        current_dict = convert_to_dictionary(d.qpos)
        position_dict["gripper"] = current_dict["gripper"]
        velocity_dict["gripper"] = 0.0

        # 发送控制
        send_position_command(d, position_dict, velocity_dict)
        mujoco.mj_step(m, d)
        viewer.sync()


def cubic_interpolation(t, joint, starting_position, desired_position, duration):
    """
    对单个关节做三次多项式插值，返回 (位置, 速度)
    """
    # 时间裁剪到 [0, T]
    if t < 0: 
        t = 0
    if t > duration: 
        t = duration

    p0 = float(starting_position[joint])
    pf = float(desired_position[joint])
    T = float(duration)

    # cubic：起止速度都为 0 的最小加加速度軌迹
    a0 = p0
    a1 = 0.0
    a2 = 3 * (pf - p0) / (T ** 2)
    a3 = -2 * (pf - p0) / (T ** 3)

    pos = a0 + a1*t + a2*(t**2) + a3*(t**3)
    vel = a1 + 2*a2*t + 3*a3*(t**2)
    return pos, vel

    
def hold_position(m, d, viewer, duration):
    current_pos = d.qpos.copy()
    current_pos_dict = convert_to_dictionary(current_pos)
    
    start_time = d.time
    while True:
        t = d.time - start_time
        if t > duration:
            break
        send_position_command(d, current_pos_dict)
        mujoco.mj_step(m, d)
        viewer.sync()

def move_to_pose_cubic(m, d, viewer, starting_position, desired_position, duration):
    """
    Move robot using cubic interpolation for each joint,
    BUT: do NOT interpolate gripper. Keep gripper at its current value.
    """

    t0 = d.time
    done = False

    # 记录需要插值的关节（不包括 gripper）
    joints_to_interpolate = [
        "shoulder_pan",
        "shoulder_lift",
        "elbow_flex",
        "wrist_flex",
        "wrist_roll",
    ]

    while not done:
        t = d.time - t0
        if t >= duration:
            done = True

        position_dict = {}
        velocity_dict = {}

        # 1) 先对 5 个大关节做 cubic 插值
        for joint in joints_to_interpolate:
            pos, vel = cubic_interpolation(t, joint, starting_position, desired_position, duration)
            position_dict[joint] = pos
            velocity_dict[joint] = vel

        # 2) gripper: 不做轨迹规划，直接保持当前角度
        current_dict = convert_to_dictionary(d.qpos)
        position_dict["gripper"] = current_dict["gripper"]
        velocity_dict["gripper"] = 0.0

        # 发送控制
        send_position_command(d, position_dict, velocity_dict)
        mujoco.mj_step(m, d)
        viewer.sync()


    # print("[CUBIC] Motion completed.")


