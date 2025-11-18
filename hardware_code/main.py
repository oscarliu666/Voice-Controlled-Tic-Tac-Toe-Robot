"""
井字棋机器人主程序
整合 game.py 逻辑和机器人控制
"""

import random


from so101_utils import hold_position,load_calibration,setup_motors,move_to_pose_cubic
from game import check_board, print_board
from cube_config import INITIAL_ARM_CONFIG, CELL_MAPPING
from cube_manager import CubeManager
from robot_control import pick_and_place_cube, reset_all_cubes,robot_clap
from audio import listen_for_move

PORT_ID = "COM15" # REPLACE WITH YOUR PORT! 
ROBOT_NAME = "follower-1" # REPLACE WITH YOUR ROBOT NAME! 

def main():
    """主游戏循环 - 整合 game.py 和机器人控制"""
    calib = load_calibration(ROBOT_NAME)
    bus = setup_motors(calib, PORT_ID)
    # 初始化 MuJoCo
    
    starting_pose = bus.sync_read("Present_Position")
    move_to_pose_cubic(bus,starting_pose, INITIAL_ARM_CONFIG,2.0)
    hold_position(bus, 2.0)
    move_to_pose_cubic(bus, INITIAL_ARM_CONFIG,starting_pose,2.0)
    bus.disable_torque()
    return

    print("Tic-Tac-Toe with SO-101 ...")
    # robot_clap(bus)
    
    # 初始化棋子管理器
    cube_manager = CubeManager()
    cube_manager.init_cubes()
    
    
    # 游戏循环（参考 game.py 的结构）
    while (1):
        ended = False
        
        location = {
            'A1', 'A2', 'A3',
            'B1', 'B2', 'B3',
            'C1', 'C2', 'C3'
        }
        
        player_marks = []
        robot_marks = []
        
        print("Game starting...")
        board = [['.' for _ in range(3)] for _ in range(3)]
        print_board(board)
        
        # 单局游戏循环
        while not ended:
            # ========== 玩家回合 (O) ==========
            # player_move = input("Pick a location: ")
            print("Please pick a location")
            player_move = listen_for_move()
            
            while player_move not in location:
                print("Invalid move")
                print("Please pick a location")
                player_move = listen_for_move()
                # player_move = input("Pick a location: ")
            
            player_marks.append(player_move)
            location.remove(player_move)
            row, col = CELL_MAPPING[player_move]
            board[row][col] = 'O'
            print_board(board)
            
            # 机器人执行玩家的落子动作
            cube_O = cube_manager.get_next_cube("O")
            if cube_O is None:
                print("[ERROR] O 棋子不够用了")
                break
            pick_and_place_cube(bus, cube_manager, cube_O, player_move)
            
            # 检查游戏状态
            ended, winner, is_draw = check_board(board)
            if ended and not is_draw:
                print("Player won! Starting next game...")
                robot_clap(bus) 
            elif is_draw:
                print("It's a draw! Starting next game...")
            
            if ended:
                break
            
            # ========== 机器人回合 (X) ==========
            robot_move = random.choice(list(location))
            print("Robot move: ", robot_move)
            robot_marks.append(robot_move)
            location.remove(robot_move)
            row, col = CELL_MAPPING[robot_move]
            board[row][col] = 'X'
            print_board(board)
            
            # 机器人执行自己的落子动作
            cube_X = cube_manager.get_next_cube("X")
            if cube_X is None:
                print("[ERROR] X")
                break
            pick_and_place_cube(bus, cube_manager, cube_X, robot_move)
            
            # 检查游戏状态
            ended, winner, is_draw = check_board(board)
            if ended and not is_draw:
                print("Robot won! Starting next game...")
            elif is_draw:
                print("It's a draw! Starting next game...")
        
        # 本局结束
        print("Player moves: ", player_marks)
        print("Robot moves: ", robot_marks)
        
        # 等待一会儿观察棋盘
        hold_position(bus, 2.0)
        
        # 复原所有棋子
        reset_all_cubes(bus, cube_manager)

        
        continue_game = input("Continue playing? (yes/no): ").strip().lower()
        if continue_game not in ['yes', 'y', '']:
            print("Thanks for playing!")
            end_pose = bus.sync_read("Present_Position")
            move_to_pose_cubic(bus,end_pose,starting_pose,2.0)
            bus.disable_torque()
            break


if __name__ == "__main__":
    main()