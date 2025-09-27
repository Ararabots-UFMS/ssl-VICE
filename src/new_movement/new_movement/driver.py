import rclpy
from rclpy.node import Node
import traceback

from new_movement.entities.Trajectory import Trajectory
from new_movement.planner import Planner
from new_movement.entities.States import Vector2D, State
from new_movement.entities.obstacles import Obstacle

from system_interfaces.msg import ControlCommand, RobotControlCommand
from system_interfaces.srv import StrategyCommand, UpdateObstacles

from strategy.blackboard import Blackboard

class PathDriver(Node):
    def __init__(self):
        super().__init__('path_driver')

        # Módulos principais
        self.blackboard = Blackboard()
        self.planner = Planner(50)

        # Dicionário para guardar dados dos robôs
        # Estrutura: { id: {'trajectory': Trajectory, 'time_offset': float, 'obstacles': list} }
        self.robot_data: dict[int, dict] = {}

        # 1. Publisher para o controle (rápido, 100Hz)
        self.publisher = self.create_publisher(ControlCommand, 'control_command', 10)
        self.control_timer = self.create_timer(0.01, self.publish_control)

        # 2. Timer para gerenciar robôs (lento e robusto, 2Hz)
        self.robot_management_timer = self.create_timer(0.5, self.manage_robots)

        # 3. Serviços para receber comandos externos
        self.update_target_service = self.create_service(StrategyCommand, 'strategy_command', self.update_target)
        self.update_obstacles_service = self.create_service(UpdateObstacles, 'update_obstacles', self.update_obstacles)

    def manage_robots(self):
        """
        Adiciona novos robôs e remove os que desapareceram. Roda em uma frequência
        baixa para ser robusto contra falhas momentâneas da visão.
        """
        if not self.blackboard.ally_robots:
            if self.robot_data:
                self.get_logger().info("Nenhum robô aliado visível, limpando dados internos.")
                self.robot_data.clear()
            return

        ally_robots_ids = self.blackboard.ally_robots.keys()

        # Adiciona novos robôs com uma trajetória "ficar parado"
        for robot_id in ally_robots_ids:
            if robot_id not in self.robot_data:
                cur_robot = self.blackboard.ally_robots[robot_id]
                cur_pos = Vector2D(cur_robot.position_x, cur_robot.position_y)
                cur_vel = Vector2D(cur_robot.velocity_x, cur_robot.velocity_y)
                cur_state = State(cur_pos, cur_vel)

                init_trajectory = Trajectory(self.planner.find(cur_state, cur_state, []))
                
                self.robot_data[robot_id] = {
                    'trajectory': init_trajectory,
                    'time_offset': 0.0,
                    'obstacles': []
                }
                self.get_logger().info(f"Robô {robot_id} inicializado no PathDriver.")

        # Remove robôs que não são mais rastreados
        for tracked_id in list(self.robot_data.keys()):
            if tracked_id not in ally_robots_ids:
                self.robot_data.pop(tracked_id)
                self.get_logger().info(f"Robô {tracked_id} removido do PathDriver por inatividade.")

    def publish_control(self):
        """
        Publica o estado atual da trajetória para cada robô. Roda em alta frequência.
        """
        if not self.robot_data:
            return

        control_command = ControlCommand()
        control_command_list = []
        
        for robot_id, robot_info in self.robot_data.items():
            robot_control_command = RobotControlCommand()

            robot_state = robot_info['trajectory'].get_state(robot_info['time_offset'])

            robot_control_command.id = robot_id
            # Envia os comandos na unidade do planejador (milímetros)
            robot_control_command.position_x = robot_state.position.x/ 1000.0
            robot_control_command.position_y = robot_state.position.y/ 1000.0
            robot_control_command.velocity_x = robot_state.velocity.x/ 1000.0
            robot_control_command.velocity_y = robot_state.velocity.y/ 1000.0

            control_command_list.append(robot_control_command)

            # Avança o tempo na trajetória, parando no final
            if robot_info['time_offset'] < robot_info['trajectory'].get_total_duration():
                robot_info['time_offset'] += 0.01

        control_command.command = control_command_list
        self.publisher.publish(control_command)

    def replan(self, robot_id: int, new_destination: State):
        """Função auxiliar para recalcular a trajetória de um robô."""
        if robot_id not in self.robot_data:
            self.get_logger().error(f"ERRO CRÍTICO: Tentativa de replanejar para o robô {robot_id}, que não existe.")
            return
            
        cur_state = self.robot_data[robot_id]['trajectory'].get_state(self.robot_data[robot_id]['time_offset'])
        new_trajectory_points = self.planner.find(cur_state, new_destination, self.robot_data[robot_id]['obstacles'])
        
        if new_trajectory_points:
            self.robot_data[robot_id]['trajectory'] = Trajectory(new_trajectory_points)
            self.robot_data[robot_id]['time_offset'] = 0.0
            self.get_logger().info(f"Nova trajetória planejada para o robô {robot_id}.")
        else:
            self.get_logger().warn(f"Planejador não conseguiu encontrar uma trajetória para o robô {robot_id}.")

    def update_target(self, request, response):
        """
        Callback do serviço que recebe o alvo da GUI.
        Esta versão é robusta e inicializa o robô se for a primeira vez que um comando é dado.
        """
        robot_id = request.id
        
        # 1. Verifica se o robô existe no Blackboard (dados da visão)
        if robot_id not in self.blackboard.ally_robots:
            self.get_logger().warn(f"Comando para robô {robot_id} ignorado: robô não está no Blackboard.")
            response.success = False
            return response

        # 2. Se for a primeira vez que recebemos um comando para este robô, inicialize-o!
        if robot_id not in self.robot_data:
            self.get_logger().info(f"Primeiro comando para robô {robot_id}. Inicializando no PathDriver...")
            cur_robot = self.blackboard.ally_robots[robot_id]
            cur_pos = Vector2D(cur_robot.position_x, cur_robot.position_y)
            cur_vel = Vector2D(cur_robot.velocity_x, cur_robot.velocity_y)
            cur_state = State(cur_pos, cur_vel)
            
            init_trajectory = Trajectory(self.planner.find(cur_state, cur_state, []))
            self.robot_data[robot_id] = {
                'trajectory': init_trajectory,
                'time_offset': 0.0,
                'obstacles': []
            }

        # 3. Agora que temos certeza que o robô existe, planejamos a nova rota.
        new_destination = State(Vector2D(request.position_x, request.position_y), 
                                Vector2D(request.velocity_x, request.velocity_y))
        self.replan(robot_id, new_destination)
        
        response.success = True
        return response
    
    def update_obstacles(self, request, response):
        # Implementar lógica de obstáculos se necessário
        response.success = True
        return response

def main(args=None):
    rclpy.init(args=args)
    node = PathDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()