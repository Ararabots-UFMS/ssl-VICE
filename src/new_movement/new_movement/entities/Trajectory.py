
from utilities.BB_steer import integrate_control_2d as integrate
from utilities.BB_steer import integrate_control_2d_at_time as integrate_t
from States import State, Vector2D
from typing import Optional, List
from Motion import MotionPath


class TrajectorySegment:
    def __init__(self, initPos: Vector2D, initVel: Vector2D, motionPath: MotionPath) -> None:
        self.initPos = initPos
        self.initVel = initVel
        self.motionPath = motionPath

        self.child: Optional["TrajectorySegment"] = None

    def add_child(self, child: "TrajectorySegment") -> None:
        """Add child segment"""
        # Check if the final of the current segment matches the start of the next segment
        local_destination: State = self.get_local_destination()
        if (
            local_destination.position.distance(child.initPos) < 1e-8
            and local_destination.velocity.distance(child.initVel) < 1e-8
        ):
            self.child = child
        else:
            raise Exception("Attempting to add a non continuous trajectory")

    def get_state(self, t: float) -> State:
        """Get the State at a time t in path"""
        if self.get_local_time() <= t:
            bb_integrate = integrate_t(self.initPos + self.initVel, self.motionPath.motion_path, t)
        else:
            bb_integrate = self.child.get_state(self.get_local_time() - t)

        return State(
            position=Vector2D(bb_integrate[0], bb_integrate[1]),
            velocity=Vector2D(bb_integrate[2], bb_integrate[3]),
        )

    def get_destination(self) -> State:
        """Get final destination State"""
        if self.child is None:
            return self.get_local_destination()

        self.child.get_destination()

    def get_local_destination(self) -> State:
        """Get local destination State"""
        # bb_integrate is a State (x, y, vx, vy)
        bb_integrate = integrate(self.initPos + self.initVel, self.motionPath.motion_path)

        return State(
            position=Vector2D(bb_integrate[0], bb_integrate[1]),
            velocity=Vector2D(bb_integrate[2], bb_integrate[3]),
        )

    def get_local_time(self) -> float:
        """Get the duration of the segment"""
        time = 0
        for p in self.motionPath.motion_path:
            time += p.duration

        return time

    def get_total_time(self) -> float:
        """Get total time of path"""
        total_time = self.get_local_time()

        if self.child is not None:
            total_time += self.child.get_total_time()

        return total_time


class Trajectory:
    def __init__(self, initial_segment: Optional[TrajectorySegment] = None):
        self.root: Optional[TrajectorySegment] = initial_segment

    def append(self, traj: TrajectorySegment) -> None:
        """Append a TrajectorySegment to the end"""
        if self.root is None:
            self.root = traj
        else:
            # Iterating the Segments utils end TrajectorySegment
            cur_segment = self.root
            while cur_segment.child is not None:
                cur_segment = cur_segment.child
            cur_segment.add_child(traj)

    def connect(self, traj: TrajectorySegment, t: float) -> None:
        """ Connect a TrajectorySegment at a time t """
        # If there is no root segment or the time to connect exceeds the trajectory total time.
        if self.root is None or self.get_total_time() < t:
            raise Exception("Trying to connect a TrajectorySegment in a unreached time space")

        cur_segment = self.root
        cur_time = self.get_total_time()
        while cur_segment.get_local_time() < cur_time:
            cur_time -= cur_segment.get_local_time()
            cur_segment = cur_segment.child

        cur_segment.motionPath.trucate(t)

        cur_segment.add_child(traj)

    def relocate(self, traj: TrajectorySegment):
        """Relocate the initial state of the first TrajectorySegment"""
        if self.root is not None and self.root.child is not None:
            traj.add_child(self.root.child)
        self.root = traj

    def get_state(self, t) -> Optional[State]:
        """Get the State at a time t in path"""
        return self.root.get_state(t) if self.root is not None else None

    def get_position(self, t) -> Optional[Vector2D]:
        return (self.get_state(t)).position

    def get_velocity(self, t) -> Optional[Vector2D]:
        return (self.get_state(t)).velocity

    def get_acceleration(self, t) -> Optional[Vector2D]:
        return (self.get_state(t)).acceleration

    def get_destination(self) -> Optional[State]:
        return self.root.get_destination() if self.root is not None else None

    def get_total_time(self) -> float:
        """Get total time of path"""
        return self.root.get_total_time() if self.root is not None else 0.0

    def to_list(
        self,
        time_step: Optional[float] = None,
        samples_size: Optional[int] = None,
        output_states: bool = False,
    ) -> List[Vector2D] | List[State]:
        """Get a list of position, or states if output_states is True, based on time_steps of samples_size if defined"""

        if (time_step is not None and samples_size is not None) or (
            time_step is None and samples_size is None
        ):
            raise ValueError(
                "to_list should have defined either time_steps or samples_size, but not both or neither."
            )

        if self.root is None:
            return []

        traj_list = []

        total_time = self.get_total_time()
        time_step = total_time / samples_size if time_step is None else time_step
        cur_time = 0.0

        while cur_time <= total_time:
            traj_list.append(
                self.get_state(cur_time) if output_states else self.get_position(cur_time)
            )
            cur_time += time_step

        # If time_step is a not a multiple of the total time, the final destination is not on traj_list
        # So is necessary to add it.
        if total_time % time_step < 1e-8:
            traj_list.append(
                self.get_destination() if output_states else self.get_destination().position
            )

        return traj_list
