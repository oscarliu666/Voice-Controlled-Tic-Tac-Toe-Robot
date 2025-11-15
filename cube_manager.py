"""
棋子管理模块
负责棋子的初始化、显示、隐藏、移动等操作
"""

import numpy as np
import mujoco

from cube_config import (
    BLOCK_HEIGHT, PILE_BASE_Z,
    X_MAIN_XY, X_SIDE_XY, O_MAIN_XY, O_SIDE_XY,
    COLOR_X, COLOR_O, PILE_PRIORITY, PILE_XY
)


class CubeManager:
    """管理所有棋子的类"""
    
    def __init__(self):
        self.cubes = []  # 存储所有棋子信息
        
    def init_cubes(self, viewer):
        """
        在 viewer.user_scn 里画出 10 个初始棋子
        - X: 3 在 (0.20, 0.075)，2 在 (0.15, 0.075)
        - O: 3 在 (0.20,-0.125)，2 在 (0.15,-0.125)
        """
        self.cubes = []
        geom_id = 0

        def add_pile(piece_type, pile_name, x, y, count):
            nonlocal geom_id
            for level in range(count):
                z = PILE_BASE_Z + level * BLOCK_HEIGHT
                pos = np.array([x, y, z], dtype=float)

                cube = {
                    "id": geom_id,
                    "type": piece_type,
                    "pile": pile_name,
                    "pos": pos,
                    "geom_id": geom_id,
                    "available": True,
                }
                self.cubes.append(cube)

                rgba = COLOR_X if piece_type == 'X' else COLOR_O

                mujoco.mjv_initGeom(
                    viewer.user_scn.geoms[geom_id],
                    type=mujoco.mjtGeom.mjGEOM_BOX,
                    size=[0.01, 0.01, BLOCK_HEIGHT / 2.0],
                    pos=pos,
                    mat=np.eye(3).flatten(),
                    rgba=rgba,
                )
                geom_id += 1

        # 初始化四个堆
        add_pile('X', 'X_main', X_MAIN_XY[0], X_MAIN_XY[1], 3)
        add_pile('X', 'X_side', X_SIDE_XY[0], X_SIDE_XY[1], 2)
        add_pile('O', 'O_main', O_MAIN_XY[0], O_MAIN_XY[1], 3)
        add_pile('O', 'O_side', O_SIDE_XY[0], O_SIDE_XY[1], 2)

        viewer.user_scn.ngeom = len(self.cubes)
        viewer.sync()

    def hide_cube(self, viewer, cube):
        """让一个 cube 从画面"消失"（挪到地下+缩小）"""
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

    def move_cube_to(self, viewer, cube, pos):
        """把 cube 的几何体移动到指定位置"""
        cube["pos"] = np.array(pos, dtype=float)
        gid = cube["geom_id"]

        rgba = COLOR_X if cube["type"] == 'X' else COLOR_O

        mujoco.mjv_initGeom(
            viewer.user_scn.geoms[gid],
            type=mujoco.mjtGeom.mjGEOM_BOX,
            size=[0.01, 0.01, BLOCK_HEIGHT / 2.0],
            pos=pos,
            mat=np.eye(3).flatten(),
            rgba=rgba,
        )
        viewer.sync()

    def get_next_cube(self, piece_type):
        """
        取下一块要用的棋子：
        - 按堆优先级：main 堆 -> side 堆
        - 同一堆中优先取 z 更高（上面）的那块
        """
        candidates = [c for c in self.cubes 
                     if c["type"] == piece_type and c["available"]]

        if not candidates:
            print(f"[WARN] 没有可用的 {piece_type} 棋子了！")
            return None

        candidates.sort(
            key=lambda c: (PILE_PRIORITY[c["pile"]], -c["pos"][2])
        )
        return candidates[0]

    def reset_cubes(self, viewer):
        """
        游戏结束后，把所有 cube 复原到初始堆叠位置
        """
        piles = {
            "X_main": [], "X_side": [],
            "O_main": [], "O_side": [],
        }
        
        for cube in self.cubes:
            piles[cube["pile"]].append(cube)

        def reset_one_pile(pile_name, base_xy):
            pile_cubes = piles[pile_name]
            pile_cubes.sort(key=lambda c: c["id"])
            x, y = base_xy

            for level, cube in enumerate(pile_cubes):
                z = PILE_BASE_Z + level * BLOCK_HEIGHT
                pos = np.array([x, y, z], dtype=float)

                cube["pos"] = pos
                cube["available"] = True

                gid = cube["geom_id"]
                rgba = COLOR_X if cube["type"] == 'X' else COLOR_O

                mujoco.mjv_initGeom(
                    viewer.user_scn.geoms[gid],
                    type=mujoco.mjtGeom.mjGEOM_BOX,
                    size=[0.01, 0.01, BLOCK_HEIGHT / 2.0],
                    pos=pos,
                    mat=np.eye(3).flatten(),
                    rgba=rgba,
                )

        # 复原四个堆
        reset_one_pile("X_main", X_MAIN_XY)
        reset_one_pile("X_side", X_SIDE_XY)
        reset_one_pile("O_main", O_MAIN_XY)
        reset_one_pile("O_side", O_SIDE_XY)

        viewer.user_scn.ngeom = len(self.cubes)
        viewer.sync()