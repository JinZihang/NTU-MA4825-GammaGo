import re
import os
import copy
from typing import Any, Iterable, List, Tuple

from numpy.lib.utils import who

SIZE = 10
P1_VICTORY_PATTERN = re.compile(r"11111")
P2_VICTORY_PATTERN = re.compile(r"22222")

PATTERN_1 = re.compile(r"211110|011112")
PATTERN_2 = re.compile(r"011110")
PATTERN_3 = re.compile(r"01110")
PATTERN_4 = re.compile(r"2011100|0011102")
PATTERN_5 = re.compile(r"010110|011010")
PATTERN_6 = re.compile(r"0110|0110")

ADV_PATTERN_1 = re.compile(r"122220|022221")
ADV_PATTERN_2 = re.compile(r"022220")
ADV_PATTERN_3 = re.compile(r"02220")
ADV_PATTERN_4 = re.compile(r"1022200|0022201")
ADV_PATTERN_5 = re.compile(r"020220|022020")
ADV_PATTERN_6 = re.compile(r"0220")

def spiral(n: int) -> List[Tuple[int, int]]:
    """
    Returns a list of coordinates to iterate a matrix of size n*n in spiral
    (outside in) order.
    """
    dx, dy = 1, 0  # Starting increments
    x, y = 0, 0    # Starting location
    matrix = [[-1]*n for _ in range(n)]
    for i in range(n**2):
        matrix[x][y] = i
        nx, ny = x + dx, y + dy
        if 0 <= nx < n and 0 <= ny < n and matrix[nx][ny] == -1:
            x, y = nx, ny
        else:
            dx, dy = -dy, dx
            x, y = x + dx, y + dy
    output = [(0, 0) for _ in range(n**2)]
    for i in range(n):
        for j in range(n):
            output[matrix[i][j]] = (i, j)
    return output

SPIRAL_ORDER = spiral(SIZE)[::-1]

def stringfy(matrix: List[List[int]]) -> str:
    string = ""
    for line in matrix:
        string += "".join(map(str, line)) + "\n"
    return string

class Board():
    """ A gomoku board, i.e., a state of the game. """

    stones = {0: ' ', 1: '●', 2: '○'}
    size = SIZE

    def __init__(self, ai_player: int) -> None:
        self._board = [[0 for _ in range(SIZE)] for _ in range(SIZE)]
        self._actual_player = 1
        self._ai_player = ai_player
        self._last_play: Tuple[str, int] = ('', 0)

    def __str__(self) -> str:
        """
        Pretty-prints the board with black and white bullets.

        Code snippet removed from github.com/zambonin/multivac
        """
        os.system('cls' if os.name == 'nt' else 'clear')

        letter_row = "     " + " ".join(
            chr(i) for i in range(65, 65 + len(self._board[0]))) + '\n'
        top_row = '   ┏' + '━' * (2 * len(self._board[0]) + 1) + '┓\n'
        bottom_row = '   ┗' + '━' * (2 * len(self._board[0]) + 1) + '┛'
        mid_rows = ""

        for row, i in zip(self._board, range(len(self._board))):
            mid_rows += '{:02d} ┃ '.format(i + 1) + ' '.join(
                Board.stones[i] for i in row) + ' ┃ {:02d}\n'.format(i + 1)

        return letter_row + top_row + mid_rows + bottom_row + '\n' + letter_row

    def place_stone(self, position: Tuple[int, int]) -> None:
        x_coord, y_coord = position
        self._last_play = (chr(x_coord + 65), y_coord + 1)
        self._board[y_coord][x_coord] = self._actual_player
        self._actual_player = 1 if self._actual_player == 2 else 2

    def is_empty(self, position: Tuple[int, int]) -> bool:
        x_coord, y_coord = position
        return self._board[y_coord][x_coord] == 0

    @property
    def last_play(self) -> Tuple[str, int]:
        return self._last_play

    def _diagonals(self) -> List[List[int]]:
        return [[self._board[SIZE - p + q - 1][q]
                 for q in range(max(p - SIZE + 1, 0), min(p + 1, SIZE))]
                for p in range(SIZE + SIZE - 1)]

    def _antidiagonals(self) -> List[List[int]]:
        return [[self._board[p - q][q]
                 for q in range(max(p - SIZE + 1, 0), min(p + 1, SIZE))]
                for p in range(SIZE + SIZE - 1)]

    def _columns(self) -> List[List[int]]:
        return [[self._board[i][j]
                 for i in range(SIZE)]
                for j in range(SIZE)]

    def victory(self) -> bool:
        whole_board = "\n".join(
            map(stringfy,
                [self._board,
                 self._diagonals(),
                 self._antidiagonals(),
                 self._columns()]))
        if P1_VICTORY_PATTERN.search(whole_board):
            return 1
        elif P2_VICTORY_PATTERN.search(whole_board):
            return 2
        else:
            return False

        # return True if P1_VICTORY_PATTERN.search(whole_board) or \
        #     P2_VICTORY_PATTERN.search(whole_board) else False

    def evaluate(self) -> int:
        """ Returns an heuristic value of the current board. """

        whole_board = "\n".join(
            map(stringfy,
                [self._board,
                 self._diagonals(),
                 self._antidiagonals(),
                 self._columns()]))

        p1_value = 0
        p2_value = 0
        if P1_VICTORY_PATTERN.search(whole_board):
            p1_value += 2**25
        elif P2_VICTORY_PATTERN.search(whole_board):
            p2_value += 2**25

        p1_value += 37 * 56 * len(PATTERN_2.findall(whole_board))
        p1_value += 56 * len(PATTERN_1.findall(whole_board))
        p1_value += 56 * len(PATTERN_3.findall(whole_board))
        p1_value += 56 * len(PATTERN_4.findall(whole_board))
        p1_value += 56 * len(PATTERN_5.findall(whole_board))
        p1_value += len(PATTERN_6.findall(whole_board))

        p2_value += 37 * 56 * len(ADV_PATTERN_2.findall(whole_board))
        p2_value += 56 * len(ADV_PATTERN_1.findall(whole_board))
        p2_value += 56 * len(ADV_PATTERN_3.findall(whole_board))
        p2_value += 56 * len(ADV_PATTERN_4.findall(whole_board))
        p2_value += 56 * len(ADV_PATTERN_5.findall(whole_board))
        p2_value += len(ADV_PATTERN_6.findall(whole_board))

        return p1_value - p2_value \
            if self._ai_player == 1 \
            else p2_value - p1_value

    def adjacents(self) -> Iterable[Any]:
        actual_board = copy.deepcopy(self)
        for i, j in SPIRAL_ORDER:
            if actual_board.is_empty((i, j)):
                actual_board.place_stone((i, j))
                yield actual_board
                actual_board._actual_player = \
                    1 if actual_board._actual_player == 2 else 2
                actual_board._board[j][i] = 0
