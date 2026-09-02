from typing import Optional

from new_movement.trapezoidal_steering import ControlIntegrator
from new_movement.entities.motion.motion_state import MotionState
from new_movement.entities.motion.motion_path import MotionPath
from utils.math_util import Vector2D

from movement_interfaces.msg import (
    TrajectorySegment as TrajectorySegmentMsg
)


class TrajectorySegment:
    """Representa um único segmento contínuo de uma trajetória."""

    def __init__(
        self, init_pos: Vector2D, init_vel: Vector2D, motion_path: MotionPath
    ) -> None:
        self.init_pos = init_pos
        self.init_vel = init_vel
        self.motion_path = motion_path
        self.integrator = ControlIntegrator()
        self.child: Optional[TrajectorySegment] = None

    def to_msg(self) -> TrajectorySegmentMsg:
        msg = TrajectorySegmentMsg()
        msg.init_pos = self.init_pos.to_msg()
        msg.init_vel = self.init_vel.to_msg()
        msg.motion_path = self.motion_path.to_msg()
        return msg

    @classmethod
    def from_msg(cls, msg: TrajectorySegmentMsg) -> "TrajectorySegment":
        return cls(
            init_pos=Vector2D.from_msg(msg.init_pos),
            init_vel=Vector2D.from_msg(msg.init_vel),
            motion_path=MotionPath.from_msg(msg.motion_path),
        )

    @property
    def initial_state(self) -> MotionState:
        """Retorna o estado inicial do segmento."""
        return MotionState(position=self.init_pos, velocity=self.init_vel)

    def add_child(self, child) -> None:
        """Adiciona um segmento filho, garantindo a continuidade da trajetória."""
        local_destination = self.get_local_destination()
        # Verifica a continuidade da posição e velocidade
        if (
            local_destination.position.distance(child.init_pos) < 1e-3
            or local_destination.velocity.distance(child.init_vel) < 1e-3
        ):
            self.child = child
        else:
            raise Exception(
                f"Attempting to add a non continuous trajectory {local_destination.position.distance(child.init_pos)}"
            )

    def get_state(self, t: float) -> MotionState:
        """Get the MotionState at a time t in path"""
        if self.get_total_duration() < t:
            return self.get_state(self.get_total_duration())
        if self.get_local_duration() >= t:
            integrated_state = self.integrator.two_dimensional_at_time(
                self.init_pos + self.init_vel, self.motion_path.motion_path, t
            )

            return MotionState(
                position=Vector2D(integrated_state[0], integrated_state[1]),
                velocity=Vector2D(integrated_state[2], integrated_state[3]),
            )
        else:
            return self.child.get_state(t - self.get_local_duration())

    def get_destination(self) -> MotionState:
        """Obtém o estado final de toda a trajetória a partir deste segmento."""
        # REFACTOR (BUG FIX): Adicionado 'return' na chamada recursiva.
        if self.child is None:
            return self.get_local_destination()
        return self.child.get_destination()

    def get_local_destination(self) -> MotionState:
        """Obtém o estado final apenas deste segmento local."""
        initial_state_tuple = (
            self.init_pos.x,
            self.init_pos.y,
            self.init_vel.x,
            self.init_vel.y,
        )
        integrated_state = self.integrator.two_dimensional(
            initial_state_tuple, self.motion_path.motion_path
        )
        return MotionState(
            position=Vector2D(integrated_state[0], integrated_state[1]),
            velocity=Vector2D(integrated_state[2], integrated_state[3]),
        )

    def get_acceleration(self, t: float) -> Optional[Vector2D]:
        """Obtém a aceleração em um tempo t no caminho."""
        local_duration = self.get_local_duration()
        if t <= local_duration:
            return self.motion_path.get_acceleration_at_time(t)
        elif self.child:
            return self.child.get_acceleration(t - local_duration)
        return None

    def get_local_duration(self) -> float:
        """Obtém a duração total deste segmento."""
        # REFACTOR: Usando sum() para mais clareza e eficiência.
        return sum(p.duration for p in self.motion_path.motion_path)

    def get_total_duration(self) -> float:
        """Obtém o tempo total da trajetória a partir deste segmento."""
        total_time = self.get_local_duration()
        if self.child is not None:
            total_time += self.child.get_total_duration()
        return total_time
