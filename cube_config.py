"""
棋子和棋盘的配置参数
包含所有位置、高度、颜色等常量定义
"""

# 机械臂参数
OPEN_GRIPPER = 10
CLOSED_GRIPPER = 0

# 方块高度相关参数
BLOCK_HEIGHT = 0.02    # 相邻两块中心的高度差
BOARD_Z = 0.014            # 棋盘上方块中心的高度
PILE_BASE_Z = 0.014        # 堆叠最底层中心高度

# 棋盘 3x3 坐标
BOARD_POSITIONS = {
    "A1": [0.25,  0.025, BOARD_Z],
    "A2": [0.25, -0.025, BOARD_Z],
    "A3": [0.25, -0.075, BOARD_Z],
    "B1": [0.20,  0.025, BOARD_Z],
    "B2": [0.20, -0.025, BOARD_Z],
    "B3": [0.20, -0.075, BOARD_Z],
    "C1": [0.15,  0.025, BOARD_Z],
    "C2": [0.15, -0.025, BOARD_Z],
    "C3": [0.15, -0.075, BOARD_Z],
}

# 四个堆的 XY 位置
X_MAIN_XY = [0.20,  0.075]    # X：3 个
X_SIDE_XY = [0.15,  0.075]    # X：2 个
O_MAIN_XY = [0.20, -0.125]    # O：3 个
O_SIDE_XY = [0.15, -0.125]    # O：2 个

# 堆的优先级（先 main 堆，再 side 堆）
PILE_PRIORITY = {
    "X_main": 0,
    "X_side": 1,
    "O_main": 0,
    "O_side": 1,
}

# 每个堆初始的棋子数量
INITIAL_PILE_COUNTS = {
    "X_main": 3,
    "X_side": 2,
    "O_main": 3,
    "O_side": 2,
}

# 每个堆的 XY 位置映射
PILE_XY = {
    "X_main": X_MAIN_XY,
    "X_side": X_SIDE_XY,
    "O_main": O_MAIN_XY,
    "O_side": O_SIDE_XY,
}

# 棋子颜色 (RGBA)
COLOR_X = [1.0, 0.3, 0.3, 0.9]  # 红色
COLOR_O = [1.0, 1.0, 1.0, 0.9]  # 白色

# 机械臂初始配置
INITIAL_ARM_CONFIG = {
    "shoulder_pan": 0.0,
    "shoulder_lift": 0.0,
    "elbow_flex": 0.0,
    "wrist_flex": 0.0,
    "wrist_roll": 0.0,
    "gripper": OPEN_GRIPPER,   
}

# 与 game.py 保持一致
CELL_MAPPING = {
    "A1": (0, 0), "A2": (0, 1), "A3": (0, 2),
    "B1": (1, 0), "B2": (1, 1), "B3": (1, 2),
    "C1": (2, 0), "C2": (2, 1), "C3": (2, 2)
}