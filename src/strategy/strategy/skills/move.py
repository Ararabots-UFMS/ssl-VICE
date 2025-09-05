from strategy.behaviour import LeafNode, TaskStatus


class MoveSkill(LeafNode):
    """
    Nó de comportamento responsável por mover um robô para uma posição alvo.

    Attributes:
        robot_id (int): Identificador do robô que deve se mover.
        target_x (float): Coordenada X de destino.
        target_y (float): Coordenada Y de destino.
        vel_x (float): Velocidade no eixo X. Default é 0.0.
        vel_y (float): Velocidade no eixo Y. Default é 0.0.
    """

    def __init__(self, name: str, robot_id: int, target_x: float, target_y: float,
                 vel_x: float = 0.0, vel_y: float = 0.0):
        """
        Inicializa o nó de movimento.

        Args:
            name (str): Nome do nó.
            robot_id (int): Identificador do robô.
            target_x (float): Coordenada X do destino.
            target_y (float): Coordenada Y do destino.
            vel_x (float, optional): Velocidade no eixo X. Default é 0.0.
            vel_y (float, optional): Velocidade no eixo Y. Default é 0.0.
        """
        super().__init__(name)
        self.robot_id = int(robot_id)
        self.target_x = float(target_x)
        self.target_y = float(target_y)
        self.vel_x = float(vel_x)
        self.vel_y = float(vel_y)

def run(self) -> TaskStatus:
    # se tiver acesso à posição atual do robô
    #pos = self.get_robot_position(self.robot_id)  # precisa implementar
    #if pos and self._is_close(pos, (self.target_x, self.target_y)):
     #   return TaskStatus.SUCCESS, self
    return TaskStatus.RUNNING, self

