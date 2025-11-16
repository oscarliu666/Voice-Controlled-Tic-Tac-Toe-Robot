import time
from so101_utils import load_calibration, move_to_pose, hold_position, setup_motors, pick_up_block, place_block,move_to_pose_cubic
from so101_inverse_kinematics import get_inverse_kinematics

import numpy as np
PORT_ID = "COM15" # REPLACE WITH YOUR PORT! 
ROBOT_NAME = "follower-1" # REPLACE WITH YOUR ROBOT NAME! 
  
# Initial joint configuration at start of simulation
initial_config = {
    'shoulder_pan': 0.0,
    'shoulder_lift': 0.0,
    'elbow_flex': 0.00,
    'wrist_flex': 0.0,
    'wrist_roll': 0.0,
    'gripper': 0          
}

def main():
    # Init
    calib = load_calibration(ROBOT_NAME)
    bus = setup_motors(calib, PORT_ID)
    starting_pose = bus.sync_read("Present_Position")        

    block1 = np.array([0.2, 0.15, 0.015])    # 起点
    block2 = np.array([0.3, 0, 0.015])  # 目标点
    block3 = np.array([0.2, -0.15, 0.015])    # 起点
    config1 = get_inverse_kinematics(block1)
    config2 = get_inverse_kinematics(block2)
    config3 = get_inverse_kinematics(block3)

    move_to_duration = 2.0 # 每次移动的持续时间（秒）
    
    move_to_pose_cubic(bus, starting_pose, config1, move_to_duration)
    move_to_pose_cubic(bus, config1, config2, move_to_duration)
    move_to_pose_cubic(bus, config2, config3, move_to_duration)
    move_to_pose_cubic(bus,  config3, starting_pose,move_to_duration)

    bus.disable_torque()


if __name__ == "__main__":
    main()

