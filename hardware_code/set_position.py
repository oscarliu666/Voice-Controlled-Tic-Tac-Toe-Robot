import time
from so101_utils import load_calibration, move_to_pose, hold_position, setup_motors

# CONFIGURATION VARIABLES
PORT_ID = "COM13" # REPLACE WITH YOUR PORT! 
ROBOT_NAME = "follower-1" # REPLACE WITH YOUR ROBOT NAME! 

# --- Specified Parameters ---
'''
This is the format of the goal position dictionary used for goal position sync write.
The gripper command takes values of 0-100, while the other joints take values of degrees, based on
the settings specified in the bus initialization.
'''
desired_position = {
    'shoulder_pan': 0.0,   # degrees
    'shoulder_lift': 0.0,
    'elbow_flex': 0.0,
    'wrist_flex': 0.0,
    'wrist_roll': 0.0,
    'gripper': 0.0          # 0-100 range
}
move_time = 2.0  # seconds to reach desired position
hold_time = 20.0  # total time to hold at 

# ------------------------

def main():
    # Load any per-robot calibration you use elsewhere
    calib = load_calibration(ROBOT_NAME)

    # Initialize the bus / motors using your existing helper
    bus = setup_motors( calib, PORT_ID)

    # Read starting pose
    starting_pose = bus.sync_read("Present_Position")
    print("Starting pose:", starting_pose)

    print(f"Moving to desired_position over {move_time:.2f}s ...")
    move_to_pose(bus, starting_pose, desired_position, move_time)

    print(f"Holding desired_position for {hold_time:.2f}s ...")
    hold_position(bus, hold_time)

    print(f"Returning to starting pose over {move_time:.2f}s ...")
    move_to_pose(bus, desired_position, starting_pose, move_time)

    print("Done.")
    
    bus.disable_torque()

if __name__ == "__main__":
    main()
