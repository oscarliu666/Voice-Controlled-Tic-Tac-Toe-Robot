import time
import mujoco
import mujoco.viewer
from so101_utils import move_to_pose, hold_position
from so101_forward_kinematics import get_forward_kinematics
import numpy as np

import time
from so101_utils import load_calibration, move_to_pose, hold_position, setup_motors

# CONFIGURATION VARIABLES
PORT_ID = "COM9" # REPLACE WITH YOUR PORT! 
ROBOT_NAME = "follower-1" # REPLACE WITH YOUR ROBOT NAME! 

# --- Specified Parameters ---
starting_configuration = {
    'shoulder_pan': -45.0,   # in radians for mujoco! 
    'shoulder_lift': 45.0,
    'elbow_flex': -45.00,
    'wrist_flex': 90.0,
    'wrist_roll': 0.0,
    'gripper': 50
}
final_configuration = {
    'shoulder_pan': 45.0,   # in radians for mujoco! 
    'shoulder_lift': 45.0,
    'elbow_flex': -45.00,
    'wrist_flex': 90.0,
    'wrist_roll': 0.0,
    'gripper': 50       
}
starting_configuration_closed = {
    'shoulder_pan': -45.0,   # in radians for mujoco! 
    'shoulder_lift': 45.0,
    'elbow_flex': -45.00,
    'wrist_flex': 90.0,
    'wrist_roll': 0.0,
    'gripper': 5
}
final_configuration_closed = {
    'shoulder_pan': 45.0,   # in radians for mujoco! 
    'shoulder_lift': 45.0,
    'elbow_flex': -45.00,
    'wrist_flex': 90.0,
    'wrist_roll': 0.0,
    'gripper': 5    
}

mid_configuration_1 = {
    'shoulder_pan':  -45.0,
    'shoulder_lift':  20.0,   
    'elbow_flex':    -10.0,
    'wrist_flex':     40.0,
    'wrist_roll':      0.0,
    'gripper':         5.0,   
}
mid_configuration_2 = {
    'shoulder_pan':   45.0,
    'shoulder_lift':  20.0,
    'elbow_flex':    -10.0,
    'wrist_flex':     40.0,
    'wrist_roll':      0.0,
    'gripper':         5.0,
}
move_time = 2.0  # seconds to reach desired position
hold_time = 2.0  # total time to hold at 

# ------------------------

def main():
    # Init
    calib = load_calibration(ROBOT_NAME)
    bus = setup_motors(calib, PORT_ID)

    # Record actual start pose (for safe return)
    starting_pose = bus.sync_read("Present_Position")
    print("Starting pose:", starting_pose)

    # Sequence (edit as needed)
    waypoints = [
        ("start-open",    starting_configuration,        move_time, hold_time),
        ("grasp-start",   starting_configuration_closed, move_time, hold_time),
        ("lift-mid1",     mid_configuration_1,           move_time, 0.2),
        ("to-mid2",       mid_configuration_2,           move_time, 0.2),
        ("approach-end",  final_configuration_closed,    move_time, 0.2),
        ("place-open",    final_configuration,           move_time, hold_time),
        ("retreat-mid2",  mid_configuration_2,           move_time, 0.2),
    ]

    try:
        # Execute sequence
        for label, target_cfg, t_move, t_hold in waypoints:
            print(f"[{label}] move {t_move:.2f}s")
            move_to_pose(bus, target_cfg, t_move)
            if t_hold and t_hold > 0:
                hold_position(bus, t_hold)

        # Return to actual start
        print(f"Return to start in {move_time:.2f}s")
        move_to_pose(bus, starting_pose, move_time)
        hold_position(bus, 0.2)
        print("Done.")
    finally:
        # Safety fallback
        try:
            move_to_pose(bus, starting_pose, move_time)
        except Exception as e:
            print("Return-to-start failed:", e)
        bus.disable_torque()


if __name__ == "__main__":
    main()


