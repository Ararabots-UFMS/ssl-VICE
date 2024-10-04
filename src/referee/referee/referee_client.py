import socket
import binascii
from referee.proto.ssl_gc_referee_message_pb2 import Referee

class Client:
    def __init__(self, ip:str, port:int):
        """Client that connects and receives messages from ssl-game-controller"""
        
        self.ip = ip
        self.port = port

    def connect(self):
        """Binds the client with ip and port and configure to UDP multicast."""


        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('', self.port))
        mreq = struct.pack("4sl", socket.inet_aton(self.ip), socket.INADDR_ANY)
        self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

    def receive(self):
        """Receive package and decode."""

        try:
            data, address = self.sock.recvfrom(1024)
        except Exception as e:
            self.get_logger().error(f"Error receiving multicast message: {str(e)}")

        referee_message = ssl_gc_referee_message_pb2.referee().ParseFromString(data)
        return referee_message

        
        
