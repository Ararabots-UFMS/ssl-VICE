# import rclpy
# from rclpy.node import Node
# import math
# import traceback

# from system_interfaces.msg import ControlCommand, TeamCommand, RobotCommand
# from system_interfaces.srv import ControlParams, SetOrientation, SetKp

# from strategy.blackboard import Blackboard
# from control.pid_controller import RobotTrajectoryController
# from control.p_controller import PController
# from new_movement.entities.States import Vector2D, State

# class Controller(Node):
#     def __init__(self):
#         super().__init__('controller')
#         self.get_logger().info('Nó Controller inicializado.')
        
#         # Dicionário para guardar as orientações alvo
#         self.target_orientations = {}

#         # Instancia os controladores com ganhos iniciais
#         self.robot_controller = RobotTrajectoryController()
#         self.orientation_controller = PController(kp=2.5, max_output=4.0)  

#         # Pub/Sub
#         self.subscriber = self.create_subscription(
#             ControlCommand, 'control_command', self.send_command, 10
#         )
#         self.publisher = self.create_publisher(TeamCommand, 'commandTopic', 10)

#         # Serviços
#         self.update_parameters_service = self.create_service(
#             ControlParams, 'update_pid', self.update_parameters
#         )
#         self.set_orientation_service = self.create_service(
#             SetOrientation, 'set_orientation', self.set_orientation_callback
#         )
#         self.update_kp_angular_service = self.create_service(
#             SetKp, 'update_kp_angular', self.update_kp_angular_callback
#         )

#         self.blackboard = Blackboard()
        
#     def send_command(self, message):
#         team_command = TeamCommand()
#         team_command.robots = []
#         team_command.is_team_color_yellow = self.blackboard.gui.is_team_color_yellow
#         commanded_robot_ids = set()

#         for robot in message.command:
#             robot_command = RobotCommand(robot_id=robot.id)
#             commanded_robot_ids.add(robot.id)
            
#             try:
#                 if robot.id not in self.blackboard.ally_robots:
#                     continue
                
#                 current_robot = self.blackboard.ally_robots[robot.id]
                
#                 # Garante que todas as unidades estão em METROS e RADIANOS
#                 current_position = Vector2D(current_robot.position_x / 1000, current_robot.position_y / 1000)
#                 current_velocity = Vector2D(current_robot.velocity_x / 1000, current_robot.velocity_y / 1000)
#                 current_state = State(current_position, current_velocity)
                
#                 # ASSUMINDO que o comando da estratégia também vem em metros. Se vier em mm, use / 1000
#                 target_position = Vector2D(robot.position_x, robot.position_y)
#                 target_velocity = Vector2D(robot.velocity_x, robot.velocity_y)
#                 target_state = State(target_position, target_velocity)
                
#                 # --- Controle de Posição (Linear) ---
#                 velocity_command = self.robot_controller.compute_trajectory_command(
#                     robot.id, target_state, current_state
#                 )
#                 robot_command.linear_velocity_x = float(velocity_command.x)
#                 robot_command.linear_velocity_y = float(velocity_command.y)

#                 # --- Controle de Orientação (Angular) ---
#                 target_orientation = self.target_orientations.get(robot.id, current_robot.orientation)
                
#                 # Chama o PController, que agora contém toda a lógica
#                 angular_velocity = self.orientation_controller.compute(
#                     target=target_orientation,
#                     current=current_robot.orientation
#                 )
#                 robot_command.angular_velocity = float(angular_velocity)
                
#                 # --- Finalização do Comando ---
#                 robot_command.orientation = current_robot.orientation
#                 robot_command.kick = 0.0
#                 team_command.robots.append(robot_command)
                
#                 # Log útil para depuração
#                 if robot.id == 0: # Log apenas para o robô 0 para não poluir o terminal
#                     self.get_logger().info(
#                         f"R{robot.id} | "
#                         f"Pos:({current_position.x:.2f},{current_position.y:.2f}) -> "
#                         f"Target:({target_position.x:.2f},{target_position.y:.2f}) | "
#                         f"VLin:({velocity_command.x:.2f},{velocity_command.y:.2f}) | "
#                         f"VAng:{angular_velocity:.2f}"
#                     )
         
#             except Exception as e:
#                 self.get_logger().error(f"Erro ao processar o robô {robot.id}: {e}\n{traceback.format_exc()}")

#         self.robot_controller.cleanup_unused_robots(commanded_robot_ids)
#         self.publisher.publish(team_command)

#     def update_parameters(self, request, response):
#         self.robot_controller.update_params(request.kp, request.ki, request.kd)
#         response.success = True
#         return response
    
#     def set_orientation_callback(self, request, response):
#         self.target_orientations[request.robot_id] = request.orientation
#         response.success = True
#         return response
    
#     def update_kp_angular_callback(self, request, response):
#         """Callback que atualiza o ganho Kp do controlador de orientação."""
#         if request.kp >= 0: # Uma verificação de segurança
#             self.orientation_controller.kp = request.kp
#             self.get_logger().info(f"Recebido da GUI via serviço: id={request.id}, x={request.position_x}, y={request.position_y}")
#             self.get_logger().info(f"Kp Angular atualizado para: {request.kp:.2f}")
#             response.success = True
#         else:
#             self.get_logger().warn(f"Valor de Kp inválido recebido: {request.kp:.2f}")
#             response.success = False
#         return response

# def main(args=None):
#     rclpy.init(args=args)
#     node = Controller()
#     rclpy.spin(node)
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
import rclpy
from rclpy.node import Node
import math
import traceback
import time 

from system_interfaces.msg import ControlCommand, TeamCommand, RobotCommand
from system_interfaces.srv import ControlParams, SetOrientation, SetKp

from strategy.blackboard import Blackboard
from control.pid_controller import RobotTrajectoryController
from control.p_controller import PController
from new_movement.entities.States import Vector2D, State

class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        self.get_logger().info('Nó Controller inicializado.')
        
        self.log_period = 0.5  # Período em segundos (0.5s = 2 logs por segundo)
        self.last_log_time = time.time()

        # Dicionário para guardar as orientações alvo
        self.target_orientations = {}

        # Instancia os controladores com ganhos iniciais
        self.robot_controller = RobotTrajectoryController()
        self.orientation_controller = PController(kp=2.5, max_output=4.0)  

        # Pub/Sub
        self.subscriber = self.create_subscription(
            ControlCommand, 'control_command', self.send_command, 10
        )
        self.publisher = self.create_publisher(TeamCommand, 'commandTopic', 10)

        # Serviços
        self.update_parameters_service = self.create_service(
            ControlParams, 'update_pid', self.update_parameters
        )
        self.set_orientation_service = self.create_service(
            SetOrientation, 'set_orientation', self.set_orientation_callback
        )
        self.update_kp_angular_service = self.create_service(
            SetKp, 'update_kp_angular', self.update_kp_angular_callback
        )

        self.blackboard = Blackboard()
        
    def send_command(self, message):
        team_command = TeamCommand()
        team_command.robots = []
        team_command.is_team_color_yellow = self.blackboard.gui.is_team_color_yellow
        commanded_robot_ids = set()

        for robot in message.command:
            robot_command = RobotCommand(robot_id=robot.id)
            commanded_robot_ids.add(robot.id)
            
            try:
                if robot.id not in self.blackboard.ally_robots:
                    continue
                
                current_robot = self.blackboard.ally_robots[robot.id]
                
                # Garante que dados da visão (Blackboard) estão em METROS
                current_position = Vector2D(current_robot.position_x / 1000.0, current_robot.position_y / 1000.0)
                current_velocity = Vector2D(current_robot.velocity_x / 1000.0, current_robot.velocity_y / 1000.0)
                current_state = State(current_position, current_velocity)
                
                # Garante que dados do comando (Estratégia/GUI) estão em METROS
                # A GUI envia em mm, então a conversão aqui é necessária.
               # Bloco corrigido, recebendo o alvo já em METROS
                target_position = Vector2D(robot.position_x, robot.position_y)
                target_velocity = Vector2D(robot.velocity_x, robot.velocity_y)
                target_state = State(target_position, target_velocity)
                
                # --- Controle de Posição (Linear) ---
                velocity_command = self.robot_controller.compute_trajectory_command(
                    robot.id, target_state, current_state
                )
                robot_command.linear_velocity_x = float(velocity_command.x)
                robot_command.linear_velocity_y = float(velocity_command.y)

                # --- Controle de Orientação (Angular) ---
                target_orientation = self.target_orientations.get(robot.id, current_robot.orientation)
                
                angular_velocity = self.orientation_controller.compute(
                    target=target_orientation,
                    current=current_robot.orientation
                )
                robot_command.angular_velocity = float(angular_velocity)
                
                # --- Finalização do Comando ---
                robot_command.orientation = current_robot.orientation
                robot_command.kick = 0.0
                team_command.robots.append(robot_command)
                
                current_time = time.time()
                if (current_time - self.last_log_time) > self.log_period:
                    # Log útil para depuração (agora controlado por tempo)
                    if robot.id == 0:
                        self.get_logger().info(
                            f"R{robot.id} | "
                            f"Pos:({current_position.x:.2f},{current_position.y:.2f}) -> "
                            f"Target:({target_position.x:.2f},{target_position.y:.2f}) | "
                            f"VLin:({velocity_command.x:.2f},{velocity_command.y:.2f}) | VAng:{angular_velocity:.2f}"
                        )
                    self.last_log_time = current_time # Atualiza o tempo do último log
                # --- FIM DA MODIFICAÇÃO ---
         
            except Exception as e:
                self.get_logger().error(f"Erro ao processar o robô {robot.id}: {e}\n{traceback.format_exc()}")

        self.robot_controller.cleanup_unused_robots(commanded_robot_ids)
        self.publisher.publish(team_command)

    def update_parameters(self, request, response):
        self.robot_controller.update_params(request.kp, request.ki, request.kd)
        response.success = True
        return response
    
    def set_orientation_callback(self, request, response):
        self.target_orientations[request.robot_id] = request.orientation
        response.success = True
        return response
    
    def update_kp_angular_callback(self, request, response):
        """Callback que atualiza o ganho Kp do controlador de orientação."""
        if request.kp >= 0:
            self.orientation_controller.kp = request.kp
            # --- CORREÇÃO DO BUG AQUI ---
            # O log foi corrigido para imprimir apenas o valor de kp recebido.
            self.get_logger().info(f"Kp Angular atualizado para: {request.kp:.2f}")
            response.success = True
        else:
            self.get_logger().warn(f"Valor de Kp inválido recebido: {request.kp:.2f}")
            response.success = False
        return response

def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()