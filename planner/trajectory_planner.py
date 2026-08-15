from collections import deque
from typing import Deque, List, Tuple

from planner.trajectory import Trajectory, Segment, SegmentType
from planner.JunctionBlender import JunctionBlender
from robot_program.api import Target
from robot_math.pose import Pose
from robot_program.instructions.instruction import *

PLAN_BUFFER_SIZE: int = 5


class TrajectoryPlanner:

    def __init__(self, buffer_size: int = 5):

        # Size of target_plan
        self.BUFFER_SIZE: int = buffer_size

        # Buffer for incoming Targets.
        self.target_buffer: Deque[Target] = deque()

        # Buffer for ready to be executed trajectories.
        self.trajectory_execution_queue: Deque[Trajectory] = deque()

        # Last planned trajectory's exit pose.
        self.last_exit_pose = Pose.identity()

        self.blender = JunctionBlender()

    def buffer_full(self) -> bool:
        return len(self.target_buffer) + len(self.trajectory_execution_queue) >= self.BUFFER_SIZE

    def process_instruction(self, instruction: InstructionNode):
        match instruction:
            case MoveJointNode():
                self.add_joint_target(instruction.target)
            case MoveCartesianJointNode():
                self.add_joint_target(instruction.target)
            case MoveCartesianLinearNode():
                self.add_cartesian_target(instruction.target)
            case WaitSecondsNode():
                self.add_delay(instruction.seconds)
            case _:
                raise TypeError("Unknown instruction.")

    def add_cartesian_target(self, target: Target) -> bool:
        # Only add if buffer not full
        if self.buffer_full():
            return False
        self.target_buffer.append(target)
        print(f"TrajPlanner: Cartesian target added.")
        self._optimize_plan()

        return True

    def add_delay(self, delay_seconds: float):
        ...
    def add_joint_target(self, target: Target):
        ...

    def _optimize_plan(self) -> None:
        """
        :return:
        """
        while len(self.target_buffer) >= 2:
            # Continuously compute the junction at current target based on last and next target.
            current_target: Target = self.target_buffer[0]
            next_target: Target = self.target_buffer[1]

            junction_poses: Tuple[Pose, Pose, List[Pose]] = self.blender.blend_junction(self.last_exit_pose, current_target.pose, next_target.pose, zone=current_target.zone)
            arc_begin_pose: Pose = junction_poses[0]
            arc_end_pose: Pose = junction_poses[1]
            arc_intermediate_poses: List[Pose] = junction_poses[2]

            # Trajectory consists of 1-2 segments.
            traj = Trajectory(target_id=current_target.name)

            # Add line segment to trajectory.
            line_segment: Segment = Segment(type=SegmentType.LINE, poses=[self.last_exit_pose, arc_begin_pose], speed=current_target.speed)
            traj.segments.append(line_segment)

            # Add arc segment to trajectory.
            if arc_intermediate_poses:
                arc_segment: Segment = Segment(type=SegmentType.ARC, poses=arc_intermediate_poses, speed=current_target.speed)
                traj.segments.append(arc_segment)

            # Trajectory ends at arc end.
            traj.end_pose = arc_end_pose
            self.trajectory_execution_queue.append(traj)

            self.last_exit_pose = arc_end_pose
            self.target_buffer.popleft()

    def flush(self) -> None:
        while self.target_buffer:
            current_target: Target = self.target_buffer.popleft()

            traj: Trajectory = Trajectory(target_id=current_target.name)
            linear_poses: List[Pose] = [self.last_exit_pose, current_target.pose]
            traj.segments.append(Segment(type=SegmentType.LINE, poses=linear_poses, speed=current_target.speed))
            traj.end_pose = current_target.pose

            self.trajectory_execution_queue.append(traj)
            self.last_exit_pose = current_target.pose
