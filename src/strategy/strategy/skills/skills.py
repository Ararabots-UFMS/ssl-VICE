from typing import List, Optional


class Skill:
    """Skill simples contendo campos opcionais.

    Campos usados:
      robot_id (obrigatório)
      target_x, target_y, vel_x, vel_y (movimento)
      angle (orientação)
      field_border, penalty_area, center_area, ball, enemy_ids, ally_ids (obstáculos)
    Apenas os campos presentes são enviados.
    """

    def __init__(self, robot_id: int):
        self.robot_id = robot_id
        # Movimento
        self.target_x: Optional[float] = None
        self.target_y: Optional[float] = None
        self.vel_x: float = 0.0
        self.vel_y: float = 0.0
        # Orientação
        self.angle: Optional[float] = None
        # Obstáculos
        self.field_border: Optional[bool] = None
        self.penalty_area: Optional[bool] = None
        self.center_area: Optional[bool] = None
        self.ball: Optional[bool] = None
        self.enemy_ids: Optional[List[int]] = None
        self.ally_ids: Optional[List[int]] = None


class Skills:
    def __init__(self, name: str):
        self.name = name

    def move_to(
        self,
        robot_id: int,
        target_x: float,
        target_y: float,
        vel_x: float = 0.0,
        vel_y: float = 0.0,
    ) -> Skill:
        s = Skill(robot_id)
        s.target_x = target_x
        s.target_y = target_y
        s.vel_x = vel_x
        s.vel_y = vel_y
        return s

    def move_with_angle(
        self,
        robot_id: int,
        target_x: float,
        target_y: float,
        angle: float,
        vel_x: float = 0.0,
        vel_y: float = 0.0,
    ) -> Skill:
        s = self.move_to(robot_id, target_x, target_y, vel_x, vel_y)
        s.angle = angle
        return s

    def set_orientation(self, robot_id: int, angle: float) -> Skill:
        s = Skill(robot_id)
        s.angle = angle
        return s

    def obstacles(
        self,
        robot_id: int,
        field_border: bool = False,
        penalty_area: bool = False,
        center_area: bool = False,
        ball: bool = False,
        enemy_ids: Optional[List[int]] = None,
        ally_ids: Optional[List[int]] = None,
    ) -> Skill:
        s = Skill(robot_id)
        s.field_border = field_border
        s.penalty_area = penalty_area
        s.center_area = center_area
        s.ball = ball
        s.enemy_ids = enemy_ids or []
        s.ally_ids = ally_ids or []
        return s

    def stop(self, robot_id: int) -> Skill:
        s = Skill(robot_id)
        s.target_x = 0.0
        s.target_y = 0.0
        s.vel_x = 0.0
        s.vel_y = 0.0
        return s


__all__ = ["Skill", "Skills"]
