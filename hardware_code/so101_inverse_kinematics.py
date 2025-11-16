import numpy as np
from so101_forward_kinematics import get_gw1,get_g12, get_g23, get_g34, get_g45,get_g5t,Rz, Ry, Rx

def get_inverse_kinematics(target_position, target_orientation=np.eye(3)):
    "Geometric appraoch specific to the so-101 arms"
    
    # Initialize the joint configuration dictionary
    joint_config = {
        'shoulder_pan': 0.0,
        'shoulder_lift': 0.0,
        'elbow_flex': 0.0,
        'wrist_flex': 0.0,
        'wrist_roll': 0.0,
        'gripper': 0.0
    }
    # 1b) Solve for θ₁
    theta1 = solve_theta1(target_position)
    # print(theta1)

    # Update dictionary
    joint_config['shoulder_pan'] = theta1-6

    # 1c)
    position ,orientation= get_wrist_flex_position(target_position)
    # print(position,orientation)

    # 1d)
    theta2, theta3 = solve_theta2_theta3(position)
    # print(theta2,theta3)
    joint_config['shoulder_lift']=theta2-3
    joint_config['elbow_flex']=theta3

    # 1e)
    orientation = get_gw1(theta1)@get_g12(theta2)@get_g23(theta3)@get_g34(0)
    theta4 = solve_theta4(orientation[0:3, 0:3])
    joint_config['wrist_flex'] = theta4+2
    # print(theta4)

    # 1f)
    joint_config['wrist_roll'] = -theta1+7

    return joint_config 


def solve_theta1(target_position):
    """
    Compute θ1 (shoulder_pan) for the SO-101 robot arm.
    target_position: [x, y, z] of end-effector in world frame.
    """
    x, y, z = target_position

    # Frame offset along x (from base to first joint axis)
    y_offset = 0.0388353

    # Compute θ1 using atan2 to handle correct quadrant
    theta1 = np.degrees(np.arctan2(y,x-y_offset))
    return -theta1

def get_wrist_flex_position(target_position):
    x, y, z = target_position
    gwt = np.eye(4)
    displacement = [x, y, z]
    rotation = Rx(0)
    gwt = np.block([[rotation, np.array(displacement).reshape(3,1)], [0, 0, 0, 1]])
    g4t = get_g45(0) @ get_g5t()
    # print(f"g45 is {get_g45(0)}")
    # print(f"g5t is {get_g5t()}")
    # print(g4t)
    # print(np.linalg.inv(g4t) )

    gw4 = gwt @ np.linalg.inv(g4t) 
    wrist_flex_position = gw4[:3, 3]
    wrist_flex_orientation = gw4[:3, :3]
    # print(wrist_flex_position,wrist_flex_orientation)
    # print(f"gw4 is {gw4}")
    return wrist_flex_position, wrist_flex_orientation


def solve_theta2_theta3(wrist_pos):

    # ----------------------------------------------------------
    # Step 1: Project the wrist position into the shoulder plane
    # ----------------------------------------------------------
    xw, yw, zw = wrist_pos
    xw=xw-0.0388353

    # Horizontal projection (distance from shoulder joint)
    r = np.sqrt(xw**2 + yw**2) -0.0303992
    # Vertical height relative to the shoulder joint
    z = zw -0.0624-0.0542

    # ----------------------------------------------------------
    # Step 2: Use the law of cosines to find elbow angle θ3
    # ----------------------------------------------------------
    origin_theta2 =  np.degrees(np.arctan2(0.028,0.11257))
    # print("origin",origin_theta2)
    link2 = np.sqrt(0.11257**2 + 0.028**2)
    link3 = 0.1349
    cos_theta3 = (-r**2 - z**2 + link2**2 + link3**2) / (2 * link2 * link3)

    # Clip to avoid numerical errors outside [-1, 1]
    cos_theta3 = np.clip(cos_theta3, -1.0, 1.0)
    temp_theta3=np.arccos(cos_theta3)
    theta3 =-np.degrees(temp_theta3)+90+origin_theta2     # elbow-up by default

    # ----------------------------------------------------------
    # Step 3: Compute shoulder angle θ2
    # ----------------------------------------------------------
    # Angle from shoulder to wrist position
    alpha = np.arctan2(z, r)
    # Internal triangle angle between link2 and link3
    beta = np.arctan2(link3 * np.sin(temp_theta3),
                      link2 - link3 * np.cos(temp_theta3))
    theta2 = 90-np.degrees(alpha) - np.degrees(beta)-origin_theta2              # elbow-up configuration
    # For elbow-down:  theta2 = alpha + beta ; theta3 = -theta3

    return theta2, theta3

def solve_theta4(wrist_orientation, desired_orientation=np.eye(3)):
    # Extract z-axes from both frames
    z_wrist = wrist_orientation[:, 1]      # current wrist z-axis
    # print(z_wrist)
    z_target = desired_orientation[:, 2]   # desired z-axis (usually [0, 0, 1])
    # print(z_target)

    x,y,z=z_wrist
    theta4=np.pi/2-np.arctan2(z,np.sqrt(x**2+y**2))
    return np.degrees(theta4)
