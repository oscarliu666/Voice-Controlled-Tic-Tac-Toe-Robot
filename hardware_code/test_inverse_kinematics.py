import time
import mujoco
import mujoco.viewer
from so101_utils import move_to_pose, hold_position
from so101_forward_kinematics import get_forward_kinematics
import numpy as np

import time
from so101_utils import load_calibration, move_to_pose, hold_position, setup_motors, pick_up_block, place_block

# CONFIGURATION VARIABLES
PORT_ID = "COM13" # REPLACE WITH YOUR PORT! 
ROBOT_NAME = "follower-1" # REPLACE WITH YOUR ROBOT NAME! 

move_time = 2.0  # seconds to reach desired position
hold_time = 2.0  # total time to hold at 

# ------------------------

def main():
    # Init
    calib = load_calibration(ROBOT_NAME)
    bus = setup_motors(calib, PORT_ID)

    starting_pose = bus.sync_read("Present_Position")

    block_position = np.array([0.2, -0.15, 0.015])    # 起点
    target_position = np.array([0.2, 0.15, 0.015])  # 目标点

    block_position[2] = block_position[2]-0.014
    target_position[2] =target_position[2]-0.014


    move_to_duration = 1.5  # 每次移动的持续时间（秒）

    pose_1 = pick_up_block(bus, starting_pose, block_position, move_to_duration)
    pose_2 = place_block(bus, pose_1, target_position, move_to_duration)
    move_to_pose(bus, pose_2, starting_pose, move_to_duration)

    block_position = np.array([0.15, -0.2, 0.015])    # 起点
    target_position = np.array([0.2, 0.15, 0.045])  # 目标点

    block_position[2] = block_position[2]-0.014
    target_position[2] =target_position[2]-0.014


    move_to_duration = 1.5  # 每次移动的持续时间（秒）

    pose_1 = pick_up_block(bus, starting_pose, block_position, move_to_duration)
    pose_2 = place_block(bus, pose_1, target_position, move_to_duration)
    move_to_pose(bus, pose_2, starting_pose, move_to_duration)
    bus.disable_torque()


if __name__ == "__main__":
    main()


