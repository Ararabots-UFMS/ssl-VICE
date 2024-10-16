import socket
import struct

class Client:
    """Client that handles the UDP multicast communication for SSL referee messages."""

    def __init__(self, ip: str, port: int, buffer_size: int = 1024):
        self.ip = ip
        self.port = port
        self.buffer_size = buffer_size
        self.sock = None

    def connect(self):
        """Sets up the multicast socket to receive data."""
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('', self.port))

        mreq = struct.pack("4sl", socket.inet_aton(self.ip), socket.INADDR_ANY)
        self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

    def receive(self):
        """Receive a message from the multicast group and return it as raw data."""
        try:
            data, _ = self.sock.recvfrom(self.buffer_size)
            return data
        except Exception as e:
            raise RuntimeError(f"Error receiving multicast message: {e}")