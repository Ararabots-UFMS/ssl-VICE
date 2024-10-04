import socket
import struct
import rclpy
import sys
import os
from rclpy.node import Node
from google.protobuf import json_format
from system_interfaces.msg import GameData
from referee.referee_client import Client

class Referee(Node):
    '''VICE Referee Node, connects and receives data from ssl-game-controler'''

    def __init__(self):
        super().__init__('refereeNode')

        # Parameters settings.
        self.declare_parameter('ip', '224.5.23.1')
        self.declare_parameter('port', 10003)

        # Verbose prints in terminal all received data.
        self.ip = self.get_parameter('ip').get_parameter_value().string_value
        self.port = self.get_parameter('port').get_parameter_value().integer_value

        # Configuração do Client
        self.client = Client(ip = self.ip, port = self.port)

        self.get_logger().info(f"Binding client on {self.ip}:{self.port}")
        self.client.connect()

        # Setting ROS publisher.
        self.publisher_ = self.create_publisher(GameData, 'referee_messages', 10)
        self.last_message = GameData()  # Variável para armazenar a última mensagem

        self.get_logger().info(f"Listening for multicast messages on {self.ip}:{self.port}")
        self.listen_to_multicast()

    def listen_to_multicast(self):
        while rclpy.ok():
            try:
                # Parse the Protobuf message
                referee_message: Referee = self.client.receive()
                
                # Setup GameData format
                msg = GameData()
                msg.stage = ssl_gc_referee_message_pb2.Referee.Stage.Name(referee_message.stage)
                msg.command = ssl_gc_referee_message_pb2.Referee.Command.Name(referee_message.command)
                msg.command_counter = referee_message.command_counter

                # Publicar a nova mensagem no formato GameData
                if(self.last_message.command_counter != msg.command_counter):
                    self.publisher_.publish(msg)
                    self.get_logger().info(f"Published new Referee message: {msg}")
                    self.last_message = msg
                # self.publisher_.publish(msg)
                # self.get_logger().info(f"Published new Referee message: {msg}")
                

            except Exception as e:
                self.get_logger().warning(f"Failed to parse Protobuf message: {str(e)}")
                    
def main(args=None):
    rclpy.init(args=args)
    node = Referee()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()