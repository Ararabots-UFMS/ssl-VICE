from dataclasses import dataclass
from States import Vector2D
from typing import Tuple

@dataclass
class MotionPrimitive:
    acceleration: Vector2D
    duration: float # seconds


@dataclass
class MotionPath:
    """Piecewise constant acceleration motion path"""

    motion_path: list[MotionPrimitive]

    def split(self, t: float) -> Tuple[list[MotionPrimitive], list[MotionPrimitive]]:
        """
        Separa um caminho em dois a partir do tempo t.
        """
        # CODIGO EXTREMAMENTE MACACO
        if t <= 0:
            return MotionPath([]), MotionPath(self.motion_path)
        
        total_time = sum(primitive.duration for primitive in self.motion_path)
        if t >= total_time:
            return MotionPath(self.motion_path), MotionPath([])
        
        elapsed_time = 0.0
        first_path: list[MotionPrimitive] = []
        second_path: list[MotionPrimitive] = []

        for primitive in self.motion_path:
            if elapsed_time + primitive.duration < t:
                first_path.append(primitive)
                elapsed_time += primitive.duration
            else:
                remaining_time = t - elapsed_time
                first_path.append(MotionPrimitive(primitive.acceleration, remaining_time))
                second_path.append(MotionPrimitive(primitive.acceleration, primitive.duration - remaining_time))
                break

        elapsed_time = 0.0
        for primitive in self.motion_path:
            if elapsed_time > t:
                second_path.append(primitive)
            elapsed_time += primitive.duration

        return MotionPath(first_path), MotionPath(second_path) 


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
