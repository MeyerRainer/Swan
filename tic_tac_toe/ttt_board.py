from dataclasses import dataclass
from typing import Optional, Tuple, List
import cv2
import numpy as np

from robot_math.pose import Pose
from tic_tac_toe.ttt_detector import DetectorOutput
from vision.opencv_camera import CameraCalibration
from vision.pose_estimator import PoseEstimatorOutput


@dataclass
class MachineMoveRequest:
    center: Optional[Pose] = None
    mark: Optional[str] = None


class GameBoard:

    def __init__(self):

        # Size of one square grid cell.
        self.grid_size: float = 0.047
        # Offset from ArUco origo to cell's (0,0) origo.
        # self.grid_offset: np.ndarray = np.array([0.082, 0.019])  # Offset from ArUco origo to first grid
        self.grid_offset: np.ndarray = np.array([0.082 - self.grid_size / 2, 0.019 - self.grid_size / 2])  # Offset from ArUco origo to first grid
        # self.grid_offset: np.ndarray = np.array([0.0585, -0.0005])  # Offset from ArUco origo to first grid. Physically measured.
        self.world_pose: Pose = Pose.identity()  # Board pose in world frame.

    @property
    def pose(self) -> Pose:
        return self.world_pose

    @pose.setter
    def pose(self, pose: Pose) -> None:
        self.world_pose = pose

    @staticmethod
    def pixel2board(px: int, py: int, cam_cal: CameraCalibration, board_pose: PoseEstimatorOutput) -> Optional[np.ndarray]:
        """ Based on an image of the game board, this function returns the XY-coordinates in the board frame in meters.
        :param px: Pixel coordinates X.
        :param py: Pixel coordinates Y.
        :param board_pose:
        :param cam_cal: Camera calibration.
        """

        # TODO: Fix name. camera2board
        board2camera: Pose = board_pose.marker_pose

        # Camera with respect to board rotation matrix.
        R_board2cam: np.ndarray = board2camera.rot_mat

        # Camera w.r.t. board translation as column vector.
        transl_board2cam: np.ndarray = board2camera.position.reshape(3, 1)

        # Inverse. (camera to board transformation)
        R_cam2board: np.ndarray = R_board2cam.T
        transl_cam2board: np.ndarray = -R_cam2board @ transl_board2cam

        # Pixel space to camera space transformation. Undistort pixel.
        pixel_coords: np.ndarray = np.array([[[float(px), float(py)]]], dtype=np.float32)
        pixel_coords_undistorted: np.ndarray = cv2.undistortPoints(src=pixel_coords, cameraMatrix=cam_cal.matrix,
                                                                   distCoeffs=cam_cal.distortion)
        # Pixel coordinates as a normalized ray in camera frame.
        camera_ray: np.ndarray = np.array([pixel_coords_undistorted[0, 0, 0], pixel_coords_undistorted[0, 0, 1], 1.0]).reshape(3, 1)

        # Rotate ray to board frame
        board_ray: np.ndarray = R_cam2board @ camera_ray  # 3x1

        # Intersection between Z=0 board plane.
        if np.abs(board_ray[2, 0]) < 1e-6:
            return None  # Ray parallel to board

        s = -transl_cam2board[2, 0] / board_ray[2, 0]
        p_board = transl_cam2board + s * board_ray

        return p_board.flatten()  # Returns [X, Y, Z] where Z should be close to zero


class TTTBoard(GameBoard):

    def __init__(self):

        super().__init__()

        # Empty board
        self.board: List[List[str]] = [[" " for _ in range(3)] for _ in range(3)]
        self.players_turn: bool = True  # Player begins.
        self.MARKS = ["O", "Board", "X"]

    def reset(self):
        self.board: List[List[str]] = [[" " for _ in range(3)] for _ in range(3)]
        self.players_turn = True
        print(f"TTT Board reset.")

    def board2grid(self, x: float, y: float) -> Optional[Tuple[int, int]]:
        """ Takes 2D coordinates in board frame and outputs the grid location.
        :param x: millimeters in board frame.
        :param y: millimeters in board frame.
        :return: Tuple of ints in range [0, 2] for grid location
        """
        xy: np.ndarray = np.array([x, y]) - self.grid_offset
        grid_x: int = int(np.floor(xy[0] / self.grid_size))
        grid_y: int = int(np.floor(xy[1] / self.grid_size))
        # Point out of grid
        if not (0 <= grid_x <= 2 and 0 <= grid_y <= 2):
            print(f"grid_x: {grid_x}\tgrid_y: {grid_y}\ty: {xy[1]:.3f}")
            return None

        return grid_x, grid_y

    def update(self, camera_calibration: CameraCalibration, ttt_detection: DetectorOutput, board_pose_estimation: PoseEstimatorOutput):
        """ Update gameboard based on detections.
        :param camera_calibration: Intrinsic camera calibration parameters.
        :param ttt_detection: YOLO-detection of game state.
        :param board_pose_estimation: ArUco-based pose estimation of game board pose.
        """

        for i in ttt_detection.indices:
            cx, cy = ttt_detection.centers[i]
            cls_id = ttt_detection.class_ids[i]
            match cls_id:
                case 0:
                    mark = "O"
                case 1:
                    mark = "Board"
                case 2:
                    mark = "X"
                case _:
                    mark = ""

            if mark not in self.MARKS:
                print(f"Invalid mark.")
                continue

            if mark == "Board":
                continue

            point_board_frame: np.ndarray = self.pixel2board(cx, cy, cam_cal=camera_calibration, board_pose=board_pose_estimation)
            if point_board_frame is None:
                print(f"Could not compute board frame coordinates.")
                continue

            grid_coords = self.board2grid(x=point_board_frame[0], y=point_board_frame[1])
            if grid_coords is None:
                print(f"TTT: Invalid board coords: X: {point_board_frame[0]:.3f}\tY: {point_board_frame[1]:.3f} for mark {mark}")
                continue

            i, j = grid_coords[0], grid_coords[1]
            if not (0 <= i <= 2 and 0 <= j <= 2):
                print(f"Out of board. (i={i}, j={j})")
                continue

            if self.board[i][j] != ' ':
                continue

            return self._add_mark(grid_coords[0], grid_coords[1], mark)

    def _add_mark(self, i: int, j: int, mark: str) -> bool:
        # New mark. Add to board.
        print(f"TTT Update: Add {mark} at X:{i}\tY:{j}")
        self.board[i][j] = mark

        if self._is_game_over():
            score: int = self._evaluate()
            draw: bool = True if score == 0 else False
            if draw:
                print(f"Game over. Draw.")
                return False
            winner = "machine" if score == -10 else "player"
            print(f"Game over. {winner} wins.")
            return False

        if mark == "O":  # Set by machine.
            self.players_turn = True
            print(f"Players turn.")
        if mark == "X":  # Set by player.
            self.players_turn = False
            machine_move = self.make_machine_move()
            if machine_move:
                print(f"Machine move: X:{machine_move[0]}\tY:{machine_move[1]}")
            else:
                print(f"Machine move solver failed.")

        return True

    # Tic-tac-toe algorithm based on Dasari Anji's implementation:
    # (https://medium.com/@dasarianjilrsa/solving-tic-tac-toe-using-python-a-comprehensive-guide-e8e51aa82f2b)
    # Function to find the best move for the player

    def make_machine_move(self) -> Optional[Tuple[int, int]]:
        """Calculates and places the machine's optimal move ('O')."""
        best_move = self.find_best_move()
        if best_move != (-1, -1):
            return best_move
        return None

    def find_best_move(self) -> Tuple[int, int]:
        """Finds the optimal move for the machine ('O')."""
        best_val = float('inf')  # 'O' wants to minimize score towards -10
        best_move = (-1, -1)

        for i in range(3):
            for j in range(3):
                if self.board[i][j] == ' ':
                    self.board[i][j] = 'O'
                    move_val = self._minimax(0, True)
                    self.board[i][j] = ' '
                    if move_val < best_val:
                        best_move = (i, j)
                        best_val = move_val

        return best_move

    def _evaluate(self) -> int:
        """Evaluates board state: +10 if 'X' wins, -10 if 'O' wins, 0 otherwise."""
        for row in range(3):
            if self.board[row][0] == self.board[row][1] == self.board[row][2] != ' ':
                return 10 if self.board[row][0] == 'X' else -10
        for col in range(3):
            if self.board[0][col] == self.board[1][col] == self.board[2][col] != ' ':
                return 10 if self.board[0][col] == 'X' else -10
        if self.board[0][0] == self.board[1][1] == self.board[2][2] != ' ':
            return 10 if self.board[0][0] == 'X' else -10
        if self.board[0][2] == self.board[1][1] == self.board[2][0] != ' ':
            return 10 if self.board[0][2] == 'X' else -10
        return 0

    def _is_game_over(self) -> bool:
        return self._evaluate() != 0 or all(self.board[i][j] != ' ' for i in range(3) for j in range(3))

    def _minimax(self, depth: int, is_maximizing: bool) -> int:
        score = self._evaluate()

        # Score adjusted for depth so machine wins faster or delays loss
        if score == 10:
            return score - depth
        if score == -10:
            return score + depth
        if self._is_game_over():
            return 0

        if is_maximizing:
            best = -float('inf')
            for i in range(3):
                for j in range(3):
                    if self.board[i][j] == ' ':
                        self.board[i][j] = 'X'
                        best = max(best, self._minimax(depth + 1, False))
                        self.board[i][j] = ' '
            return best
        else:
            best = float('inf')
            for i in range(3):
                for j in range(3):
                    if self.board[i][j] == ' ':
                        self.board[i][j] = 'O'
                        best = min(best, self._minimax(depth + 1, True))
                        self.board[i][j] = ' '
            return best
