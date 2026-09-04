from typing import Optional, List, Union

from movement.entities.motion.motion_state import MotionState
from movement.entities.trajectory.trajectory_segment import TrajectorySegment

from utils.math_util import Vector2D

from movement_interfaces.msg import (
    Trajectory as TrajectoryMsg
)

class Trajectory:
    """Gerencia uma sequência de TrajectorySegments."""

    def __init__(self, initial_segment: Optional[TrajectorySegment] = None):
        self.root: Optional[TrajectorySegment] = initial_segment
        # REFACTOR: Adicionado ponteiro para a cauda para otimizar o append para O(1).
        self.tail: Optional[TrajectorySegment] = initial_segment
        # Via point que o BypassSolver usou, para servir de warm start no próximo ciclo.
        self.via_state: Optional[MotionState] = None
        # PlanningStatus de quem gerou esta trajetória. Fica aqui, e não no Planner,
        # porque o Planner é compartilhado entre as threads que planejam cada robô.
        self.status = None
        if self.root:
            # Encontra a cauda inicial se a trajetória não for vazia
            node = self.root
            while node.child:
                node = node.child
            self.tail = node

    def to_msg(self, robot_id: int) -> TrajectoryMsg:
        msg = TrajectoryMsg()
        msg.robot_id = int(robot_id)

        current_segment = self.root
        while current_segment:
            msg.segments.append(current_segment.to_msg())
            current_segment = current_segment.child

        return msg

    @classmethod
    def from_msg(cls, msg: TrajectoryMsg) -> "Trajectory":
        trajectory = cls()
        for seg_msg in msg.segments:
            trajectory.append(TrajectorySegment.from_msg(seg_msg))
        return trajectory

    def append(self, segment: TrajectorySegment) -> None:
        """Anexa um TrajectorySegment ao final da trajetória."""
        if self.root is None:
            self.root = segment
            self.tail = segment
        else:
            self.tail.add_child(segment)
            self.tail = segment

    def connect(self, new_segment: TrajectorySegment, t: float) -> None:
        """
        Divide a trajetória em um tempo t e conecta um novo segmento,
        descartando o restante da trajetória original.
        """
        total_duration = self.get_total_duration()
        if self.root is None or t > total_duration or t < 0:
            raise ValueError(
                f"Tempo de conexão fora dos limites da trajetória. {t, total_duration}"
            )

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

    def get_state(self, t: float) -> Optional[MotionState]:
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

    def get_destination(self) -> Optional[MotionState]:
        return self.root.get_destination() if self.root is not None else None

    def get_total_duration(self) -> float:
        """Obtém a duração total da trajetória."""
        return self.root.get_total_duration() if self.root is not None else 0.0

    def to_list(
        self,
        time_step: Optional[float] = None,
        samples_size: Optional[int] = None,
        start_time: Optional[float] = None,
        end_time: Optional[float] = None,
        output_states: bool = False,
    ) -> Union[List[Vector2D], List[MotionState]]:
        """
        Retorna uma lista de posições ou estados amostrados da trajetória dentro de um intervalo de tempo.
        Deve ser fornecido `time_step` OU `samples_size`.
        """
        if (time_step is not None) == (samples_size is not None):
            raise ValueError(
                "Defina `time_step` ou `samples_size`, mas não ambos ou nenhum."
            )

        if self.root is None:
            return []

        # Define os tempos de início e fim padrões se não forem fornecidos
        total_duration = self.get_total_duration()
        t_start = start_time if start_time is not None else 0.0
        t_end = end_time if end_time is not None else total_duration

        # Garante que os tempos informados estejam dentro dos limites e ordens lógicas
        t_start = max(0.0, min(t_start, total_duration))
        t_end = max(t_start, min(t_end, total_duration))
        
        interval_duration = t_end - t_start
        traj_list = []

        # Caso onde o intervalo de amostragem é zero (mesmo ponto no tempo)
        if interval_duration == 0:
            state = self.get_state(t_start)
            return [state] if output_states else [state.position]

        # Calcula o time_step baseado na quantidade de amostras desejadas para o intervalo
        if samples_size is not None:
            time_step = (
                interval_duration / (samples_size - 1) if samples_size > 1 else interval_duration
            )

        # Varre a trajetória a partir do t_start usando o time_step
        cur_time = t_start
        # Usamos uma pequena tolerância para evitar problemas de precisão de ponto flutuante no loop
        while cur_time < t_end - 1e-9:
            item = (
                self.get_state(cur_time)
                if output_states
                else self.get_position(cur_time)
            )
            traj_list.append(item)
            cur_time += time_step

        # Garante que o ponto final exato do intervalo seja sempre incluído
        end_state = self.get_state(t_end)
        traj_list.append(
            end_state if output_states else end_state.position
        )

        return traj_list
