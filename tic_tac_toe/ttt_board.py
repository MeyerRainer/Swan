from dataclasses import dataclass
from typing import Optional, Tuple, List, Callable
import cv2
import numpy as np

from robot_math.pose import Pose
from tic_tac_toe.ttt_detector import DetectorOutput
from vision.opencv_camera import CameraCalibration
from vision.pose_estimator import PoseEstimatorOutput


@dataclass
class MachineMoveRequest:
    mark_world_pose: Optional[Pose] = None
    mark: Optional[str] = None


class GameBoard:

    def __init__(self):

        # Size of one square grid cell.
        self.grid_size: float = 0.047

        # Offset from ArUco origo to cell's (0,0) origo.
        self.grid_offset: np.ndarray = np.array([0.082 - self.grid_size / 2, 0.019 - self.grid_size / 2])

        # Board pose in world frame.
        self.world_pose: Pose = Pose.identity()

    @property
    def pose(self) -> Pose:
        return self.world_pose

    @pose.setter
    def pose(self, pose: Pose) -> None:
        self.world_pose = pose

    @staticmethod
    def image2board(px: int, py: int, cam_cal: CameraCalibration, board_pose: PoseEstimatorOutput) -> Optional[np.ndarray]:
        """ Based on an image of the game board, this function returns the XYZ-coordinates in the board frame in meters.
        :param px: Pixel coordinates X.
        :param py: Pixel coordinates Y.
        :param board_pose: Board pose with respect to world frame.
        :param cam_cal: Camera calibration.
        """
        # Camera w.r.t. board transformation.
        board2camera: Pose = board_pose.marker_pose.inverse()
        # Camera -> board rotation matrix.
        R_cam2board: np.ndarray = board2camera.rot_mat
        # Camera -> board translation vector.
        transl_cam2board: np.ndarray = board2camera.position.reshape(3, 1)

        # Pixel space to camera space transformation. Undistort pixel. Shape=(1, 1, 2)
        pixel_coords: np.ndarray = np.array([[[float(px), float(py)]]], dtype=np.float32)
        pixel_coords_undistorted: np.ndarray = cv2.undistortPoints(
            src=pixel_coords, cameraMatrix=cam_cal.matrix, distCoeffs=cam_cal.distortion)
        # Pixel coordinates as a normalized ray in camera frame.
        camera_ray: np.ndarray = np.array([pixel_coords_undistorted[0, 0, 0], pixel_coords_undistorted[0, 0, 1], 1.0]).reshape(3, 1)

        # Rotate ray to board frame
        board_ray: np.ndarray = R_cam2board @ camera_ray  # 3x1

        # Intersection between Z=0 board plane.
        if np.abs(board_ray[2, 0]) < 1e-6:
            return None  # Ray parallel to board

        s = -transl_cam2board[2, 0] / board_ray[2, 0]
        p_board = transl_cam2board + s * board_ray

        return p_board.flatten()  # Returns [X, Y, Z] where Z should be close to zero.


class TTTBoard(GameBoard):

    def __init__(self, msg_callback: Callable[[str], None]):

        super().__init__()

        # Empty board
        self.board: List[List[str]] = [[" " for _ in range(3)] for _ in range(3)]
        self.players_turn: bool = True  # Player begins.
        self.MARKS = ["O", "Board", "X"]

        # Messages
        self.msg_callback: Callable[[str], None] = msg_callback

    def reset(self):
        self.board: List[List[str]] = [[" " for _ in range(3)] for _ in range(3)]
        self.players_turn = True
        self.msg_callback("Game reset.")

    def board2grid(self, x: float, y: float) -> Optional[Tuple[int, int]]:
        """ Takes 2D coordinates in board frame and outputs the grid location.
        :param x: millimeters in board frame.
        :param y: millimeters in board frame.
        :return: Tuple of ints in range [0, 2] for grid location
        """
        xy: np.ndarray = np.array([x, y]) - self.grid_offset
        i: int = int(np.floor(xy[0] / self.grid_size))
        j: int = int(np.floor(xy[1] / self.grid_size))

        # Point out of grid.
        if not (0 <= i <= 2 and 0 <= j <= 2):
            return None

        return i, j

    def grid2board(self, i: int, j: int) -> np.ndarray:
        """ Inputs grid location and outputs XYZ-coordinates in board frame as meters.
        :param i: Grid number in x-direction in range [0, 2]
        :param j: Grid number in y-direction in range [0, 2]
        :return: XYZ coordinates of given grid in board frame, meters.
        """
        x: float = self.grid_offset[0] + (0.5 + i) * self.grid_size
        y: float = self.grid_offset[1] + (0.5 + j) * self.grid_size
        return np.array([x, y, 0.], dtype=np.float64)

    def board2world(self, board_coords: np.ndarray):
        """ Transforms 3D vector from board frame to world frame.
        :param board_coords: XYZ coordinates in board frame, meters.
        :return: XYZ coordinates in world frame, meters.
        """
        return self.world_pose.vector_mult(board_coords)

    def update(self, camera_calibration: CameraCalibration, ttt_detection: DetectorOutput, board_pose_estimation: PoseEstimatorOutput):
        """ Update gameboard based on detections.
        :param camera_calibration: Intrinsic camera calibration parameters.
        :param ttt_detection: YOLO-detection of game state.
        :param board_pose_estimation: ArUco-based pose estimation of the game board. Board w.r.t. camera.
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

            # Invalid or uninteresting mark, ignore.
            if mark not in self.MARKS or mark == "Board":
                continue

            point_board_frame: np.ndarray = self.image2board(cx, cy, cam_cal=camera_calibration, board_pose=board_pose_estimation)
            if point_board_frame is None:
                self.msg_callback(f"Could not compute board frame coordinates.")
                continue

            grid_coords = self.board2grid(x=point_board_frame[0], y=point_board_frame[1])
            if grid_coords is None:
                self.msg_callback(f"Invalid board coords: X:{point_board_frame[0]:.3f} Y:{point_board_frame[1]:.3f} for mark {mark}")
                continue

            i, j = grid_coords[0], grid_coords[1]
            if not (0 <= i <= 2 and 0 <= j <= 2):
                self.msg_callback(f"Grid out of board. (i={i}, j={j})")
                continue

            if self.board[i][j] != ' ':
                continue

            mark_center_world_coords: np.ndarray = self._add_mark(grid_coords[0], grid_coords[1], mark)
            if mark_center_world_coords is not None:
                # X's pose with respect to world frame.
                # x_world_pose: Pose = Pose.from_rot_mat(pos=mark_center_world_coords, R=camera_calibration.extrinsic.rot_mat @ board_pose_estimation.marker_pose.rot_mat)
                # print(x_world_pose)
                # return MachineMoveRequest(mark_world_pose=x_world_pose, mark=mark)
                return MachineMoveRequest(mark_world_pose=Pose.from_position(mark_center_world_coords), mark=mark)

    def _add_mark(self, i: int, j: int, mark: str) -> Optional[np.ndarray]:
        # New mark. Add to board.
        self.msg_callback(f"Board updated. Mark {mark} added at X:{i} Y:{j}.")
        self.board[i][j] = mark

        if self._is_game_over():
            score: int = self._evaluate()
            draw: bool = True if score == 0 else False
            if draw:
                self.msg_callback(f"Game over. Draw.")
                return None
            winner = "machine" if score == -10 else "player"
            self.msg_callback(f"Game over. {winner} wins.")
            return None

        if mark == "O":  # Set by machine.
            self.players_turn = True
            self.msg_callback(f"Players turn.")
        if mark == "X":  # Set by player.
            self.players_turn = False
            machine_move = self.make_machine_move()
            if machine_move:
                board_coords: np.ndarray = self.grid2board(machine_move[0], machine_move[1])
                mark_center_world_coords: np.ndarray = self.board2world(board_coords)
                # self.msg_callback(f"Board coords: {board_coords}")
                self.msg_callback(f"Machine move to grid X:{machine_move[0]:.3f}Y:{machine_move[1]:.3f}.")
                self.msg_callback(f"World coordinates: X:{mark_center_world_coords[0]:.3f}Y:{mark_center_world_coords[1]:.3f}Z:{mark_center_world_coords[2]:.3f}.")
                return mark_center_world_coords
                #return MachineMoveRequest(center=Pose.from_position(mark_center_world_coords), mark="O")
            else:
                self.msg_callback(f"Machine move solver failed.")

        return None

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
