import socket
import struct
import rclpy
import sys
import os
from rclpy.node import Node
from std_msgs.msg import String
from google.protobuf import json_format

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), 'proto')))

try:
    import ssl_gc_referee_message_pb2
except ImportError as e:
    print(f"Error importing Protobuf: {e}")
    sys.exit(1)

MULTICAST_GROUP = '224.5.23.1'
MULTICAST_PORT = 10003
BUFFER_SIZE = 1024

class MulticastListener(Node):
    def __init__(self):
        super().__init__('multicast_listener')
        self.publisher_ = self.create_publisher(String, 'referee_messages', 10)
        self.last_message = None  # Variável para armazenar a última mensagem

        # Configuração do socket multicast
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('', MULTICAST_PORT))
        mreq = struct.pack("4sl", socket.inet_aton(MULTICAST_GROUP), socket.INADDR_ANY)
        self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

        self.get_logger().info(f"Listening for multicast messages on {MULTICAST_GROUP}:{MULTICAST_PORT}")
        self.listen_to_multicast()

    def listen_to_multicast(self):
        while rclpy.ok():
            try:
                data, address = self.sock.recvfrom(BUFFER_SIZE)
                referee_message = ssl_gc_referee_message_pb2.Referee()

                try:
                    referee_message.ParseFromString(data)
                    json_message = json_format.MessageToJson(referee_message)

                    # Verifique se a mensagem é diferente da última antes de publicar
                    if json_message != self.last_message:
                        self.last_message = json_message  # Atualize a última mensagem

                        # Publicar a nova mensagem
                        msg = String()
                        msg.data = json_message
                        self.publisher_.publish(msg)
                        self.get_logger().info("Published new Referee message.")

                except Exception as e:
                    self.get_logger().warning(f"Failed to parse Protobuf message: {str(e)}")
                    self.get_logger().info(f"Message in hex: {data.hex()}")
                    
            except Exception as e:
                self.get_logger().error(f"Error receiving multicast message: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = MulticastListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()