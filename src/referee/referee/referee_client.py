import socket
import struct

class RefereeClient:
    """Client that handles the UDP multicast communication for SSL referee messages.

    Supports joining a multicast group on an explicit local interface IP when needed
    (useful when Docker or system networking requires specifying the interface).
    """

    def __init__(self, ip: str, port: int, buffer_size: int):
        self.ip = ip
        self.port = port
        self.buffer_size = buffer_size
        self.sock = None

    def connect(self, interface_ip: str | None = None):
        """Sets up the multicast socket to receive data.

        If `interface_ip` is provided it will be used to join the multicast group on
        that local interface. Otherwise INADDR_ANY ('0.0.0.0') is used.
        """
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        # Allow multiple sockets to reuse the same address
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        # Bind to the port on all interfaces
        self.sock.bind(("", self.port))

        group = socket.inet_aton(self.ip)
        if interface_ip:
            local_ip = socket.inet_aton(interface_ip)
        else:
            local_ip = socket.inet_aton("0.0.0.0")

        # Use 4s4s packing which is portable for two IPv4 addresses
        mreq = struct.pack("4s4s", group, local_ip)
        self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

    def receive(self):
        """Receive a message from the multicast group and return it as raw data."""
        try:
            data, addr = self.sock.recvfrom(self.buffer_size)
            return data, addr
        except socket.timeout:
            return None
        except Exception as e:
            raise RuntimeError(f"Error receiving multicast message: {e}")
