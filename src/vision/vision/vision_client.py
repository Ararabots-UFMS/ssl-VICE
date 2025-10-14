import socket
import struct
from typing import Optional

from vision.proto.messages_robocup_ssl_wrapper_pb2 import SSL_WrapperPacket


class Client:
    def __init__(self, ip: str, port: int, interface_ip: Optional[str] = None, timeout: float = 0.0):
        """UDP multicast client for ssl-vision.

        ip: multicast group address (e.g. 224.5.23.2)
        port: multicast UDP port
        interface_ip: specific local interface IP to join from. If None/empty, uses INADDR_ANY.
        timeout: socket timeout in seconds (0 => non-blocking)
        """
        self.ip = ip
        self.port = port
        self.interface_ip = interface_ip or ""
        self.timeout = timeout
        self.sock: Optional[socket.socket] = None

    def connect(self):
        """Configure socket for receiving multicast packets.

        We bind to INADDR_ANY so the kernel delivers packets from any interface.
        Then we join the multicast group on the chosen interface (or all interfaces when supported).
        """
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        # Allow multiple sockets to bind same (addr,port) (common pattern for multicast receive)
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        except OSError:
            pass

        # Bind to all interfaces (important for receiving on Linux)
        sock.bind(("", self.port))

        # Build membership request
        group = socket.inet_aton(self.ip)
        if self.interface_ip:
            try:
                local_ip = socket.inet_aton(self.interface_ip)
            except OSError:
                # Fallback to INADDR_ANY if provided interface can't be parsed
                local_ip = struct.pack("!I", socket.INADDR_ANY)
        else:
            local_ip = struct.pack("!I", socket.INADDR_ANY)

        mreq = group + local_ip
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

        # Disable loopback of our own outgoing multicast (we normally don't send, but be explicit)
        try:
            sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 0)
        except OSError:
            pass

        # Non-blocking / timeout configuration so we don't stall the ROS executor
        if self.timeout == 0:
            sock.setblocking(False)
        else:
            sock.settimeout(self.timeout)

        self.sock = sock

    def receive(self):
        """Try to receive and decode one SSL_WrapperPacket.

        Returns None if no data is available (non-blocking) so caller can continue.
        """
        if self.sock is None:
            raise RuntimeError("Client socket not connected. Call connect() first.")
        try:
            # Max packet size for SSL-Vision is safely under 4096 bytes; allocate a bit more than old 2048.
            data, _ = self.sock.recvfrom(4096)
        except (BlockingIOError, socket.timeout):
            return None
        except OSError:
            # Transient error; report no data
            return None

        packet = SSL_WrapperPacket()
        try:
            packet.ParseFromString(data)
        except Exception:
            # If parsing fails, ignore this packet
            return None
        return packet
