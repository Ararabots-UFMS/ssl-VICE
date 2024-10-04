from grsim_messenger.protobuf.grSim_Packet_pb2 import grSim_Packet
from grsim_messenger.protobuf.grSim_Commands_pb2 import grSim_Robot_Command

from rclpy.node import Node
import rclpy
from rclpy.executors import MultiThreadedExecutor

from grsim_messenger.grsim_sender import grSimSender
from system_interfaces.msg import TeamCommand
from utils.topic_subscriber import TopicSubscriber
from grsim_messenger.inverse_kinematics import apply_inverse_kinematics

class grsim_publisher(Node):
    def __init__(self) -> None:
        super().__init__('grsim_publisher')
        
        # Parameters settings.
        self.declare_parameter('ip', '127.0.0.1')
        self.declare_parameter('port', 20011)
        
        self.ip = self.get_parameter('ip').get_parameter_value().string_value
        self.port = self.get_parameter('port').get_parameter_value().integer_value
        
        self.publisher = grSimSender(ip=self.ip, port=self.port)
        
        self.subscriber = TopicSubscriber('command_subs', TeamCommand, 'CommandTopic')
        
        self.publish_timer = self.create_timer(0.016, self.send_to_grsim)
        
        executor = MultiThreadedExecutor()
        executor.add_node(self)
        executor.add_node(self.subscriber)  
        executor.spin()
        
    def send_to_grsim(self):
        
        command = self.subscriber.get_message()
        
        if command is not None:
            packet = self.get_packet(command)
            self.publisher.send(packet)
        
    def get_packet(self, command):
        packet = grSim_Packet()
        packet.commands.isteamyellow = command.isteamyellow
        packet.commands.timestamp = 0
        
        for robot in command.robots:
            
            wheel_speed = apply_inverse_kinematics(robot.linear_velocity_x, robot.linear_velocity_y, robot.angular_velocity, 0)
            
            robot_command = grSim_Robot_Command()
            robot_command.id = robot.robot_id
            robot_command.wheelsspeed = 1
            robot_command.kickspeedx = 0
            robot_command.kickspeedz = 0
            robot_command.veltangent = 0
            robot_command.velnormal = 0
            robot_command.velangular = 0 
            robot_command.spinner = 0
            robot_command.wheel1 = wheel_speed[0]
            robot_command.wheel2 = wheel_speed[1]
            robot_command.wheel3 = wheel_speed[2]
            robot_command.wheel4 = wheel_speed[3]
            
            packet.commands.robot_commands.append(robot_command)
            
        return packet
    
def main(args=None):
    rclpy.init(args=args)
    node = grsim_publisher()
    
    rclpy.spin(node)

if __name__ == '__main__':
    main()