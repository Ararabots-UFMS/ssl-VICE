import socket
import struct
import rclpy
import sys
import os
from rclpy.node import Node
from google.protobuf import json_format
from system_interfaces.msg import GameData
#from referee.proto.ssl_gc_referee_message_pb2.proto import Referee

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), 'proto')))

try:
    import ssl_gc_referee_message_pb2
except ImportError as e:
    print(f"Error importing Protobuf: {e}")
    sys.exit(1)

class Referee(Node):
    '''VICE Referee Node, connects and receives data from ssl-game-controler'''

    def __init__(self):
        super().__init__('refereeNode')

        # Parameters settings.
        self.declare_parameter('ip', '224.5.23.1')
        self.declare_parameter('port', 10003)
        self.declare_parameter('buffer_size', 1024)

        # Verbose prints in terminal all received data.
        self.ip = self.get_parameter('ip').get_parameter_value().string_value
        self.port = self.get_parameter('port').get_parameter_value().integer_value
        self.buffer_size = self.get_parameter('buffer_size').get_parameter_value().integer_value

        self.publisher_ = self.create_publisher(GameData, 'referee_messages', 10)
        self.last_message = GameData()  # Variável para armazenar a última mensagem

        # Configuração do socket multicast
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('', self.port))
        mreq = struct.pack("4sl", socket.inet_aton(self.ip), socket.INADDR_ANY)
        self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

        self.get_logger().info(f"Listening for multicast messages on {self.ip}:{self.port}")
        self.listen_to_multicast()

    def listen_to_multicast(self):
        while rclpy.ok():
            try:
                data, address = self.sock.recvfrom(self.buffer_size)
                referee_message = ssl_gc_referee_message_pb2.Referee()

                try:
                    # Parse the Protobuf message
                    referee_message.ParseFromString(data)

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
                    
            except Exception as e:
                self.get_logger().error(f"Error receiving multicast message: {str(e)}")
    
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