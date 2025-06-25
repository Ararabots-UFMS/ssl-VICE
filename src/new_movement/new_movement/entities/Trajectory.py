from __future__ import annotations
from typing import Optional, List, Union
from .States import State, Vector2D
from .Motion import MotionPath
from ..utils.BB_steer import (
    integrate_control_2d_at_time as integrate_t,
    integrate_control_2d as integrate,
)


class TrajectorySegment:
    """Representa um único segmento contínuo de uma trajetória."""

    def __init__(self, init_pos: Vector2D, init_vel: Vector2D, motion_path: MotionPath) -> None:
        self.init_pos = init_pos
        self.init_vel = init_vel
        self.motion_path = motion_path
        self.child: Optional[TrajectorySegment] = None

    @property
    def initial_state(self) -> State:
        """Retorna o estado inicial do segmento."""
        return State(position=self.init_pos, velocity=self.init_vel)

    def add_child(self, child: TrajectorySegment) -> None:
        """Adiciona um segmento filho, garantindo a continuidade da trajetória."""
        local_destination = self.get_local_destination()
        # Verifica a continuidade da posição e velocidade
        if (
            local_destination.position.distance(child.init_pos) > 1e-6
            or local_destination.velocity.distance(child.init_vel) > 1e-6
        ):
            raise ValueError("Tentativa de adicionar um segmento de trajetória não contínuo.")
        self.child = child

    def get_state(self, t: float) -> State:
        """Obtém o estado (posição, velocidade) em um tempo t no caminho."""
        if t < 0:
            raise ValueError("O tempo não pode ser negativo.")

        local_duration = self.get_local_duration()

        # REFACTOR (BUG FIX): A lógica da recursão foi corrigida.
        # Se o tempo t está dentro deste segmento, calcula localmente.
        if t <= local_duration:
            # Assumindo que a função de integração espera um tuple (x, y, vx, vy)
            initial_state_tuple = (
                self.init_pos.x,
                self.init_pos.y,
                self.init_vel.x,
                self.init_vel.y,
            )
            bb_integrate = integrate_t(initial_state_tuple, self.motion_path.motion_path, t)
            return State(
                position=Vector2D(bb_integrate[0], bb_integrate[1]),
                velocity=Vector2D(bb_integrate[2], bb_integrate[3]),
            )
        # Se o tempo t for maior, delega para o segmento filho com o tempo restante.
        elif self.child:
            return self.child.get_state(t - local_duration)
        else:
            # Se o tempo for maior que a duração e não há filho, retorna o estado final.
            return self.get_local_destination()

    def get_destination(self) -> State:
        """Obtém o estado final de toda a trajetória a partir deste segmento."""
        # REFACTOR (BUG FIX): Adicionado 'return' na chamada recursiva.
        if self.child is None:
            return self.get_local_destination()
        return self.child.get_destination()

    def get_local_destination(self) -> State:
        """Obtém o estado final apenas deste segmento local."""
        initial_state_tuple = (self.init_pos.x, self.init_pos.y, self.init_vel.x, self.init_vel.y)
        bb_integrate = integrate(initial_state_tuple, self.motion_path.motion_path)
        return State(
            position=Vector2D(bb_integrate[0], bb_integrate[1]),
            velocity=Vector2D(bb_integrate[2], bb_integrate[3]),
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


class Trajectory:
    """Gerencia uma sequência de TrajectorySegments."""

    def __init__(self, initial_segment: Optional[TrajectorySegment] = None):
        self.root: Optional[TrajectorySegment] = initial_segment
        # REFACTOR: Adicionado ponteiro para a cauda para otimizar o append para O(1).
        self.tail: Optional[TrajectorySegment] = initial_segment
        if self.root:
            # Encontra a cauda inicial se a trajetória não for vazia
            node = self.root
            while node.child:
                node = node.child
            self.tail = node

    def append(self, segment: TrajectorySegment) -> None:
        """Anexa um TrajectorySegment ao final da trajetória."""
        if self.root is None:
            self.root = segment
            self.tail = segment
        else:
            # REFACTOR: Operação O(1) usando self.tail
            self.tail.add_child(segment)
            self.tail = segment

    def connect(self, new_segment: TrajectorySegment, t: float) -> None:
        """
        Divide a trajetória em um tempo t e conecta um novo segmento,
        descartando o restante da trajetória original.
        """
        total_duration = self.get_total_duration()
        if self.root is None or t > total_duration or t < 0:
            raise ValueError("Tempo de conexão fora dos limites da trajetória.")

        # REFACTOR (BUG FIX): Lógica de busca de segmento e truncamento corrigida.
        current_segment = self.root
        time_elapsed = 0.0

        while current_segment:
            segment_duration = current_segment.get_local_duration()
            if time_elapsed + segment_duration >= t:
                # O ponto de conexão está neste segmento
                local_time_to_truncate = t - time_elapsed

                # Trunca o motion path do segmento atual
                current_segment.motion_path.truncate(local_time_to_truncate)

                # Descarta o restante da trajetória
                current_segment.child = None
                self.tail = current_segment  # O segmento atual é a nova cauda

                # Adiciona o novo segmento
                self.append(new_segment)
                return

            time_elapsed += segment_duration
            current_segment = current_segment.child

    def relocate(self, new_root_segment: TrajectorySegment):
        """Substitui o primeiro segmento (root), mantendo o restante da trajetória."""
        if self.root and self.root.child:
            new_root_segment.add_child(self.root.child)
        self.root = new_root_segment
        if not self.root.child:  # Se a trajetória só tinha um segmento
            self.tail = self.root

    def get_state(self, t: float) -> Optional[State]:
        """Obtém o estado em um tempo t na trajetória."""
        return self.root.get_state(t) if self.root is not None else None

    def get_position(self, t: float) -> Optional[Vector2D]:
        state = self.get_state(t)
        return state.position if state else None

    def get_velocity(self, t: float) -> Optional[Vector2D]:
        state = self.get_state(t)
        return state.velocity if state else None

    def get_acceleration(self, t: float) -> Optional[Vector2D]:
        state = self.get_state(t)
        return state.acceleration if state else None


    def get_destination(self) -> Optional[State]:
        return self.root.get_destination() if self.root is not None else None

    def get_total_duration(self) -> float:
        """Obtém a duração total da trajetória."""
        return self.root.get_total_duration() if self.root is not None else 0.0

    def to_list(
        self,
        time_step: Optional[float] = None,
        samples_size: Optional[int] = None,
        output_states: bool = False,
    ) -> Union[List[Vector2D], List[State]]:
        """
        Retorna uma lista de posições ou estados amostrados da trajetória.
        Deve ser fornecido `time_step` OU `samples_size`.
        """
        if (time_step is not None) == (samples_size is not None):
            raise ValueError("Defina `time_step` ou `samples_size`, mas não ambos ou nenhum.")

        if self.root is None:
            return []

        traj_list = []
        total_time = self.get_total_duration()

        if total_time == 0:
            # Se a trajetória tem duração zero, retorna apenas o estado inicial
            initial_state = self.get_state(0)
            return [initial_state] if output_states else [initial_state.position]

        if samples_size is not None:
            time_step = total_time / (samples_size - 1) if samples_size > 1 else total_time

        cur_time = 0.0
        # REFACTOR (BUG FIX): Lógica de loop e amostragem robusta.
        while cur_time < total_time:
            item = self.get_state(cur_time) if output_states else self.get_position(cur_time)
            traj_list.append(item)
            cur_time += time_step

        # Garante que o ponto final exato seja sempre incluído
        destination_state = self.get_destination()
        traj_list.append(destination_state if output_states else destination_state.position)

        return traj_list
