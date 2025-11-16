import random

def check_board(board):
    lines = []

    for r in range(3):
        lines.append([board[r][0], board[r][1], board[r][2]])
        lines.append([board[0][r], board[1][r], board[2][r]])

    lines.append([board[0][0], board[1][1], board[2][2]])
    lines.append([board[0][2], board[1][1], board[2][0]])

    for line in lines:
        if line[0] != '.' and line[0] == line[1] == line[2]:
            return True, line[0], False
    
    for r in range(3):
        for c in range(3):
            if board[r][c] == '.':
                return False, None, False
            
    return True, None, True

def print_board(board):
    for i in range(3):
        print(board[i])

def main():
    mapping = {
        "A1": (0, 0), "A2": (0, 1), "A3": (0, 2),
        "B1": (1, 0), "B2": (1, 1), "B3": (1, 2),
        "C1": (2, 0), "C2": (2, 1), "C3": (2, 2)
    }

    while True:
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

        while not ended:
            player_move = input("Pick a location: ")

            while player_move not in location:
                print("Invalid move")
                player_move = input("Pick a location: ")
            
            player_marks.append(player_move)
            location.remove(player_move)
            row, col = mapping[player_move]
            board[row][col] = 'O'
            print_board(board)
            ended, winner, is_draw = check_board(board)
            if ended and not is_draw:
                print("Player won! Starting next game...")
            elif is_draw:
                print("It's a draw! Starting next game...")
            
            if ended:
                break

            robot_move = random.choice(list(location))
            print("Robot move: ", robot_move)
            robot_marks.append(robot_move)
            location.remove(robot_move)
            row, col = mapping[robot_move]
            board[row][col] = 'X'
            print_board(board)
            ended, winner, is_draw = check_board(board)
            if ended and not is_draw:
                print("Robot won! Starting next game...")
            elif is_draw:
                print("It's a draw! Starting next game...")

        print("Player moves: ", player_marks)
        print("Robot moves: ", robot_marks)

if __name__ == "__main__":
    main()