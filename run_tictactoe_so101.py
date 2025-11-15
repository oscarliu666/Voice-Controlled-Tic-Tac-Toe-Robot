import time
import random
import numpy as np
import mujoco
import mujoco.viewer

from so101_mujoco_utils import (
    set_initial_pose,
    move_to_pose_cubic,
    hold_position,
    convert_to_dictionary,
    send_position_command,
)
from so101_inverse_kinematics import get_inverse_kinematics

OPEN_GRIPPER = 15
CLOSED_GRIPPER = 0  # 或 40.0，看你模型闭合程度

initial_config = {
    "shoulder_pan": 0.0,
    "shoulder_lift": 0.0,
    "elbow_flex": 0.0,
    "wrist_flex": 0.0,
    "wrist_roll": 0.0,
    "gripper": 0.0,
}


# =========================
# 1. 井字棋逻辑
# =========================

def check_board(board):
    """检查棋盘是否结束：返回 (ended, winner, is_draw)"""
    lines = []

    # 行和列
    for r in range(3):
        lines.append([board[r][0], board[r][1], board[r][2]])  # 行
        lines.append([board[0][r], board[1][r], board[2][r]])  # 列

    # 对角线
    lines.append([board[0][0], board[1][1], board[2][2]])
    lines.append([board[0][2], board[1][1], board[2][0]])

    for line in lines:
        if line[0] != '.' and line[0] == line[1] == line[2]:
            return True, line[0], False  # 有赢家

    # 是否还有空格
    for r in range(3):
        for c in range(3):
            if board[r][c] == '.':
                return False, None, False  # 没结束

    # 棋盘下满且没人赢 → 平局
    return True, None, True


def print_board_ascii(board):
    """在终端打印棋盘"""
    print("   1   2   3")
    for i, row_name in enumerate(['A', 'B', 'C']):
        row = board[i]
        print(f"{row_name}  {row[0]} | {row[1]} | {row[2]}")
        if i < 2:
            print("  ---+---+---")
    print()


# =========================
# 2. 棋盘 & 堆叠参数
# =========================

# 方块高度相关（根据你 workspace 图：0.014, 0.043, 0.071... 大约差 0.0285）
BLOCK_HEIGHT = 0.0285          # 相邻两块中心的高度差
BOARD_Z = 0.014                # 棋盘上方块中心的高度
PILE_BASE_Z = 0.014            # 堆叠最底层中心高度（和第一块 z 一致）

# 棋盘 3x3 坐标（你给的）
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

# 四个堆的位置（XY）
X_MAIN_XY = [0.20,  0.075]    # X：3 个
X_SIDE_XY = [0.15,  0.075]    # X：2 个
O_MAIN_XY = [0.20, -0.125]    # O：3 个
O_SIDE_XY = [0.15, -0.125]    # O：2 个

# 堆的优先级：先 main 堆，再 side 堆
PILE_PRIORITY = {
    "X_main": 0,
    "X_side": 1,
    "O_main": 0,
    "O_side": 1,
}

# 每个堆一开始有多少个块
INITIAL_PILE_COUNTS = {
    "X_main": 3,
    "X_side": 2,
    "O_main": 3,
    "O_side": 2,
}

# 每个堆的 XY 位置（方便复原时用）
PILE_XY = {
    "X_main": X_MAIN_XY,
    "X_side": X_SIDE_XY,
    "O_main": O_MAIN_XY,
    "O_side": O_SIDE_XY,
}

# 全局保存每个 cube 的信息
cubes = []  # 每个元素: {"id","type","pile","pos","geom_id","available"}


# =========================
# 3. 初始化 10 个棋子（堆叠）
# =========================

def init_pile_cubes(viewer):
    """
    在 viewer.user_scn 里画出 10 个初始棋子：
    - X: 3 在 (0.20, 0.075)，2 在 (0.15, 0.075)
    - O: 3 在 (0.20,-0.125)，2 在 (0.15,-0.125)
    红色 = X，白色 = O
    """
    global cubes
    cubes = []
    geom_id = 0

    def add_pile(piece_type, pile_name, x, y, count):
        nonlocal geom_id
        for level in range(count):
            z = PILE_BASE_Z + level * BLOCK_HEIGHT
            pos = np.array([x, y, z], dtype=float)

            cube = {
                "id": geom_id,
                "type": piece_type,   # 'X' 或 'O'
                "pile": pile_name,    # 'X_main', 'X_side', 'O_main', 'O_side'
                "pos": pos,
                "geom_id": geom_id,
                "available": True,
            }
            cubes.append(cube)

            # 画出这个 cube
            color_X = [1.0, 0.3, 0.3, 0.9]  # 红色
            color_O = [1.0, 1.0, 1.0, 0.9]  # 白色
            rgba = color_X if piece_type == 'X' else color_O

            mujoco.mjv_initGeom(
                viewer.user_scn.geoms[geom_id],
                type=mujoco.mjtGeom.mjGEOM_BOX,
                size=[0.01, 0.01, BLOCK_HEIGHT / 2.0],  # 半长宽高
                pos=pos,
                mat=np.eye(3).flatten(),
                rgba=rgba,
            )
            geom_id += 1

    # X: 3 + 2
    add_pile('X', 'X_main', X_MAIN_XY[0], X_MAIN_XY[1], 3)
    add_pile('X', 'X_side', X_SIDE_XY[0], X_SIDE_XY[1], 2)

    # O: 3 + 2
    add_pile('O', 'O_main', O_MAIN_XY[0], O_MAIN_XY[1], 3)
    add_pile('O', 'O_side', O_SIDE_XY[0], O_SIDE_XY[1], 2)

    viewer.user_scn.ngeom = len(cubes)
    viewer.sync()


def hide_cube(viewer, cube):
    """
    让一个 cube 从画面“消失”（挪到地下+缩小）
    """
    gid = cube["geom_id"]
    mujoco.mjv_initGeom(
        viewer.user_scn.geoms[gid],
        type=mujoco.mjtGeom.mjGEOM_BOX,
        size=[1e-6, 1e-6, 1e-6],
        pos=[0.0, 0.0, -1.0],
        mat=np.eye(3).flatten(),
        rgba=[0, 0, 0, 0],
    )
    viewer.sync()


def move_cube_to_board(viewer, cube, board_pos):
    """
    把 cube 的几何体移动到棋盘格子中心（place 之后的效果）
    """
    cube["pos"] = np.array(board_pos, dtype=float)
    gid = cube["geom_id"]

    color_X = [1.0, 0.3, 0.3, 0.9]
    color_O = [1.0, 1.0, 1.0, 0.9]
    rgba = color_X if cube["type"] == 'X' else color_O

    mujoco.mjv_initGeom(
        viewer.user_scn.geoms[gid],
        type=mujoco.mjtGeom.mjGEOM_BOX,
        size=[0.01, 0.01, BLOCK_HEIGHT / 2.0],
        pos=board_pos,
        mat=np.eye(3).flatten(),
        rgba=rgba,
    )
    viewer.sync()


def get_next_cube(piece_type):
    """
    取下一块要用的棋子：
    - 按堆优先级：main 堆 -> side 堆
    - 同一堆中优先取 z 更高（上面）的那块
    """
    candidates = [c for c in cubes if c["type"] == piece_type and c["available"]]

    if not candidates:
        print(f"[WARN] 没有可用的 {piece_type} 棋子了！")
        return None

    candidates.sort(
        key=lambda c: (PILE_PRIORITY[c["pile"]], -c["pos"][2])
    )
    return candidates[0]

def reset_pile_cubes(viewer):
    """
    游戏结束后，把所有 cube 复原到初始堆叠位置：
    - 仍然按每堆的类型和 pile 名字
    - 同一堆内按 id 从小到大决定底层到顶层
    """
    # 按堆把 cube 分好组
    piles = {
        "X_main": [],
        "X_side": [],
        "O_main": [],
        "O_side": [],
    }
    for cube in cubes:
        piles[cube["pile"]].append(cube)

    # 一堆一堆复原
    def reset_one_pile(pile_name, base_xy):
        pile_cubes = piles[pile_name]
        # 按 id 排序：创建顺序 = 初始从底到顶的顺序
        pile_cubes.sort(key=lambda c: c["id"])
        x, y = base_xy

        for level, cube in enumerate(pile_cubes):
            z = PILE_BASE_Z + level * BLOCK_HEIGHT
            pos = np.array([x, y, z], dtype=float)

            cube["pos"] = pos
            cube["available"] = True    # 全部重新变成“可用”

            gid = cube["geom_id"]
            color_X = [1.0, 0.3, 0.3, 0.9]
            color_O = [1.0, 1.0, 1.0, 0.9]
            rgba = color_X if cube["type"] == 'X' else color_O

            mujoco.mjv_initGeom(
                viewer.user_scn.geoms[gid],
                type=mujoco.mjtGeom.mjGEOM_BOX,
                size=[0.01, 0.01, BLOCK_HEIGHT / 2.0],
                pos=pos,
                mat=np.eye(3).flatten(),
                rgba=rgba,
            )

    # 对四个堆分别复原
    reset_one_pile("X_main", X_MAIN_XY)
    reset_one_pile("X_side", X_SIDE_XY)
    reset_one_pile("O_main", O_MAIN_XY)
    reset_one_pile("O_side", O_SIDE_XY)

    # 更新场景计数，防止显示不出来
    viewer.user_scn.ngeom = len(cubes)
    viewer.sync()



# =========================
# 4. 机械臂 pick & place
# =========================
def move_gripper_only(m, d, viewer, target_angle, duration=0.5):
    """
    只控制 gripper，从当前角度平滑插值到 target_angle，
    其余关节保持调用时的姿态不变。
    """
    start_time = d.time
    start_dict = convert_to_dictionary(d.qpos)
    start_angle = start_dict["gripper"]

    while True:
        t = d.time - start_time
        if t > duration:
            break

        alpha = min(t / duration, 1.0)

        # 其他关节固定在开始时的姿态，只更新 gripper
        cmd = dict(start_dict)
        cmd["gripper"] = (1.0 - alpha) * start_angle + alpha * target_angle

        send_position_command(d, cmd)
        mujoco.mj_step(m, d)
        viewer.sync()



def pick_and_place_cube(
    m,
    d,
    viewer,
    cube,
    cell_name,
    hover_height=0.05,
    move_duration=1.0,
):
    """
    用 SO-101 把指定 cube 从它当前堆叠位置拿起来（pick），
    再放到 cell_name 对应棋盘格子（place）。
    """
    if cell_name not in BOARD_POSITIONS:
        print(f"[WARN] 格子 {cell_name} 不在 BOARD_POSITIONS 里")
        return

    # 1) 堆叠位置（从 cube 记录里读）
    stack_pos = cube["pos"].copy()
    stack_pos_above = stack_pos.copy()
    stack_pos_above[2] += hover_height

    # 2) 棋盘位置
    board_pos = np.array(BOARD_POSITIONS[cell_name], dtype=float)
    board_pos_above = board_pos.copy()
    board_pos_above[2] += hover_height

    # 3) 当前关节配置（从 d.qpos 反算成字典）
    current_dict = convert_to_dictionary(d.qpos)

    # ---------- 1. 去堆叠上方（张开） ----------
    config_stack_above = get_inverse_kinematics(stack_pos_above, np.eye(3))
    config_stack_above["gripper"] = OPEN_GRIPPER

    move_to_pose_cubic(
        m,
        d,
        viewer,
        starting_position=current_dict,
        desired_position=config_stack_above,
        duration=move_duration,
    )

    # ---------- 2. 下到方块中心（保持张开） ----------
    config_stack_down = get_inverse_kinematics(stack_pos, np.eye(3))
    config_stack_down["gripper"] = OPEN_GRIPPER

    move_to_pose_cubic(
        m,
        d,
        viewer,
        starting_position=config_stack_above,
        desired_position=config_stack_down,
        duration=move_duration / 2.0,
    )

# ---------- 3. 在方块上方闭合夹爪 ✅ ----------
    move_gripper_only(m, d, viewer, target_angle=CLOSED_GRIPPER, duration=0.6)


    # ---------- 4. 提回堆叠上方（保持闭合） ----------
    # 现在 d.qpos 里已经是闭合的角度了，所以从 d.qpos 读一遍
    stack_above_closed = convert_to_dictionary(d.qpos)

    move_to_pose_cubic(
        m,
        d,
        viewer,
        starting_position=convert_to_dictionary(d.qpos),
        desired_position=stack_above_closed,
        duration=move_duration / 4.0,
    )

    # ---------- 5. 堆叠上方 → 棋盘上方（闭合） ----------
    config_board_above = get_inverse_kinematics(board_pos_above, np.eye(3))
    config_board_above["gripper"] = CLOSED_GRIPPER

    move_to_pose_cubic(
        m,
        d,
        viewer,
        starting_position=stack_above_closed,
        desired_position=config_board_above,
        duration=move_duration,
    )

    # ---------- 6. 下到棋盘格子（闭合） ----------
    config_board_down = get_inverse_kinematics(board_pos, np.eye(3))
    config_board_down["gripper"] = CLOSED_GRIPPER

    move_to_pose_cubic(
        m,
        d,
        viewer,
        starting_position=config_board_above,
        desired_position=config_board_down,
        duration=move_duration / 2.0,
    )

    # ---------- 7. 在棋盘处张开夹爪（放下 ✅） ----------
    move_gripper_only(m, d, viewer, target_angle=OPEN_GRIPPER, duration=0.4)

    # 让这块 cube 出现在棋盘格子位置
    move_cube_to_board(viewer, cube, board_pos)

    # ✅ 标记：这个棋子现在已经在棋盘上，不再在堆里
    cube["available"] = False


    # 让这块 cube 出现在棋盘格子位置
    move_cube_to_board(viewer, cube, board_pos)

    # ---------- 8. 抬回棋盘上方 ----------
    config_board_above_open = get_inverse_kinematics(board_pos_above, np.eye(3))
    config_board_above_open["gripper"] = OPEN_GRIPPER

    move_to_pose_cubic(
        m,
        d,
        viewer,
        starting_position=convert_to_dictionary(d.qpos),
        desired_position=config_board_above_open,
        duration=move_duration / 2.0,
    )

    hold_position(m, d, viewer, 0.2)

def return_cube_to_pile(
    m,
    d,
    viewer,
    cube,
    target_pos,
    hover_height=0.05,
    move_duration=1.0,
):
    """
    用 SO-101 把已经在棋盘上的 cube 再次 pick & place 回某个堆叠位置 target_pos。
    target_pos 是一个 3D 坐标 [x, y, z]，对应某个堆里的某一层。
    """

    # 1) 棋盘上的当前位置（cube 已经在棋盘上）
    board_pos = cube["pos"].copy()
    board_pos_above = board_pos.copy()
    board_pos_above[2] += hover_height

    # 2) 目标堆叠位置
    pile_pos = np.array(target_pos, dtype=float)
    pile_pos_above = pile_pos.copy()
    pile_pos_above[2] += hover_height

    # 3) 当前关节配置
    current_dict = convert_to_dictionary(d.qpos)

    # ---------- 1. 去棋盘上方（张开） ----------
    config_board_above = get_inverse_kinematics(board_pos_above, np.eye(3))
    config_board_above["gripper"] = OPEN_GRIPPER

    move_to_pose_cubic(
        m,
        d,
        viewer,
        starting_position=current_dict,
        desired_position=config_board_above,
        duration=move_duration,
    )

    # ---------- 2. 下到棋盘方块中心（张开） ----------
    config_board_down = get_inverse_kinematics(board_pos, np.eye(3))
    config_board_down["gripper"] = OPEN_GRIPPER

    move_to_pose_cubic(
        m,
        d,
        viewer,
        starting_position=config_board_above,
        desired_position=config_board_down,
        duration=move_duration / 2.0,
    )

    # ---------- 3. 在棋盘上闭合夹爪（抓住这个棋子） ----------
    move_gripper_only(m, d, viewer, target_angle=CLOSED_GRIPPER, duration=0.4)

    # 把棋盘上的这个 geom 隐身（等会儿在堆里重新画出来）
    hide_cube(viewer, cube)

    # ---------- 4. 提回棋盘上方（保持闭合） ----------
    board_above_closed = convert_to_dictionary(d.qpos)
    move_to_pose_cubic(
        m,
        d,
        viewer,
        starting_position=convert_to_dictionary(d.qpos),
        desired_position=board_above_closed,
        duration=move_duration / 2.0,
    )

    # ---------- 5. 棋盘上方 → 堆叠上方（闭合） ----------
    config_pile_above = get_inverse_kinematics(pile_pos_above, np.eye(3))
    config_pile_above["gripper"] = CLOSED_GRIPPER

    move_to_pose_cubic(
        m,
        d,
        viewer,
        starting_position=board_above_closed,
        desired_position=config_pile_above,
        duration=move_duration,
    )

    # ---------- 6. 下到堆叠位置（闭合） ----------
    config_pile_down = get_inverse_kinematics(pile_pos, np.eye(3))
    config_pile_down["gripper"] = CLOSED_GRIPPER

    move_to_pose_cubic(
        m,
        d,
        viewer,
        starting_position=config_pile_above,
        desired_position=config_pile_down,
        duration=move_duration / 2.0,
    )

    # ---------- 7. 在堆叠处张开夹爪（放下） ----------
    move_gripper_only(m, d, viewer, target_angle=OPEN_GRIPPER, duration=0.4)

    # 在堆叠处重新显示这个 cube
    move_cube_to_board(viewer, cube, pile_pos)

    # 标记：这个 cube 又回到堆里了（下局可以再用）
    cube["available"] = True

    # ---------- 8. 抬回堆叠上方 ----------
    config_pile_above_open = get_inverse_kinematics(pile_pos_above, np.eye(3))
    config_pile_above_open["gripper"] = OPEN_GRIPPER

    move_to_pose_cubic(
        m,
        d,
        viewer,
        starting_position=convert_to_dictionary(d.qpos),
        desired_position=config_pile_above_open,
        duration=move_duration / 2.0,
    )

    hold_position(m, d, viewer, 0.2)

def reset_all_cubes(m, d, viewer):
    """
    游戏结束后调用：
    让机械臂把所有在棋盘上的棋子按原本的 3+2 / 3+2 堆叠方式送回各自的堆。
    没用过的块（还在堆里的）就不动它。
    """
    print("开始复原所有棋子到初始堆叠位置...")

    # 不再强制回 home，机械臂保持在游戏结束时的姿态
    # 可以稍微 hold 一下让状态稳定
    hold_position(m, d, viewer, 0.2)

    # 对每个堆分别处理（顺序：X_main → X_side → O_main → O_side）
    pile_order = ["X_main", "X_side", "O_main", "O_side"]

    for pile_name in pile_order:
        initial_count = INITIAL_PILE_COUNTS[pile_name]
        xy = PILE_XY[pile_name]

        # 这个堆一共有哪些 cube（包括在堆里和在棋盘上的）
        pile_cubes = [c for c in cubes if c["pile"] == pile_name]

        # 还留在堆里的（available=True）
        available_cubes = [c for c in pile_cubes if c["available"]]

        # 已经被拿到棋盘上的（available=False）
        board_cubes = [c for c in pile_cubes if not c["available"]]

        if not board_cubes:
            # 这个堆没有被动过，不用复原
            continue

        # 目前堆里有多少层
        current_layers = len(available_cubes)

        # 按任意顺序逐个把棋盘上的 cube 送回堆顶
        # 每送回一个，就往上叠一层
        for i, cube in enumerate(board_cubes):
            level = current_layers + i  # 从现有层数往上叠
            target_pos = np.array(
                [xy[0], xy[1], PILE_BASE_Z + level * BLOCK_HEIGHT],
                dtype=float,
            )
            return_cube_to_pile(m, d, viewer, cube, target_pos=target_pos)

        print("所有棋子已复原到初始堆叠。")

    # ---------- 复原完，把机械臂平滑移动回初始姿态 ----------


    # 从当前姿态 -> home，用 cubic 做一个平滑轨迹
    current_dict = convert_to_dictionary(d.qpos)
    move_to_pose_cubic(
        m,
        d,
        viewer,
        starting_position=current_dict,
        desired_position=initial_config,
        duration=2.0,   # 可以自己调快/慢
    )

    # 稍微停一下
    hold_position(m, d, viewer, 0.5)




# =========================
# 5. 主程序：一局井字棋 + MuJoCo 仿真
# =========================

def main():
    m = mujoco.MjModel.from_xml_path("model/scene.xml")
    d = mujoco.MjData(m)

    # 一个比较舒适的初始姿态（你也可以按 test_move 里的改）

    set_initial_pose(d, initial_config)

    mapping_rc = { 
        "A1": (0, 0),
        "A2": (0, 1),
        "A3": (0, 2),
        "B1": (1, 0),
        "B2": (1, 1),
        "B3": (1, 2),
        "C1": (2, 0),
        "C2": (2, 1),
        "C3": (2, 2),
    }
    all_locations = set(mapping_rc.keys())

    with mujoco.viewer.launch_passive(m, d) as viewer:
        print("MuJoCo Tic-Tac-Toe with SO-101 (piles & pick-and-place) starting...")

        # 初始化 10 个堆叠棋子
        init_pile_cubes(viewer)

        # 先 step 一会儿让画面稳定
        for _ in range(10):
            mujoco.mj_step(m, d)
            viewer.sync()

        board = [["." for _ in range(3)] for _ in range(3)]
        available = set(all_locations)
        player_marks = []
        robot_marks = []
        ended = False

        print_board_ascii(board)

        while viewer.is_running() and not ended:
            # ---------- 玩家回合 (O) ----------
            player_move = input("玩家下子（例如 A1 / B2 / C3）: ").strip().upper()
            while player_move not in available:
                print("无效位置或已被占用，请重新输入。")
                player_move = input("玩家下子（例如 A1 / B2 / C3）: ").strip().upper()

            player_marks.append(player_move)
            available.remove(player_move)
            r, c = mapping_rc[player_move]
            board[r][c] = "O"
            print_board_ascii(board)

            cube_O = get_next_cube("O")
            if cube_O is None:
                print("[ERROR] O 棋子不够用了，直接结束。")
                break
            pick_and_place_cube(m, d, viewer, cube_O, cell_name=player_move)

            ended, winner, is_draw = check_board(board)
            if ended:
                if is_draw:
                    print("平局！")
                else:
                    print("玩家获胜！")
                break

            # ---------- 机器人回合 (X) ----------
            if not available:
                break

            robot_move = random.choice(list(available))
            print("机器人落子:", robot_move)
            robot_marks.append(robot_move)
            available.remove(robot_move)
            r, c = mapping_rc[robot_move]
            board[r][c] = "X"
            print_board_ascii(board)

            cube_X = get_next_cube("X")
            if cube_X is None:
                print("[ERROR] X 棋子不够用了，直接结束。")
                break
            pick_and_place_cube(m, d, viewer, cube_X, cell_name=robot_move)

            ended, winner, is_draw = check_board(board)
            if ended:
                if is_draw:
                    print("平局！")
                else:
                    print("机器人获胜！")
                break

        print("本局结束。")
        print("玩家走子顺序:", player_marks)
        print("机器人走子顺序:", robot_marks)

        # 先停一会看看棋盘
        hold_position(m, d, viewer, 2.0)

        # 让机械臂把所有棋子复原到初始堆叠位置
        reset_all_cubes(m, d, viewer)

        # 复原完再停一会儿
        hold_position(m, d, viewer, 3.0)




if __name__ == "__main__":
    main()
