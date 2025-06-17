from dataclasses import dataclass
from States import Vector2D


@dataclass
class MotionPrimitive:
    acceleration: Vector2D
    duration: float


@dataclass
class MotionPath:
    """Piecewise constant acceleration motion path"""

    motion_path: list[MotionPrimitive]

    def truncate(self, t: float) -> None:
        """
        Trunca o caminho de movimento para que sua duração total seja 't'.

        - Se 't' for menor ou igual a 0, o caminho se torna vazio.
        - Se 't' for maior ou igual à duração total atual do caminho,
          o caminho permanece inalterado.
        - Modifica o caminho de movimento no lugar.
        """
        if t <= 0:
            self.motion_path = []
            return

        total_time = sum(mp.duration for mp in self.motion_path)
        if t >= total_time:
            return  # nothing to truncate

        elapsed_time = 0.0
        new_path: list[MotionPrimitive] = []

        for mp in self.motion_path:
            if elapsed_time + mp.duration < t:
                new_path.append(mp)
                elapsed_time += mp.duration
            else:
                remaining_time = t - elapsed_time
                new_path.append(MotionPrimitive(mp.acceleration, remaining_time))
                break

        self.motion_path = new_path
