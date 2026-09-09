import socket
import struct
import logging
from typing import Optional
from rclpy.logging import get_logger

from vision.proto.messages_robocup_ssl_wrapper_pb2 import SSL_WrapperPacket

MAX_PACKET_SIZE = 4096  # SSL-Vision packets are safely under this size

class Client:
    def __init__(self, ip: str, port: int, interface_ip: Optional[str] = None, 
                timeout: float = 0.0, logger: Optional[logging.Logger] = None):
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
        self.logger = logger or logging.getLogger(__name__)


    def connect(self):
        # Create UDP socket for multicast reception
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)

        # Allow multiple processes to bind to the same address/port (important for multiple ROS nodes on same machine)
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        except OSError as e:
            self.logger.warning(f"SO_REUSEADDR not supported: {e}")

        # Bind to all interfaces (important for receiving on Linux)
        sock.bind(("", self.port))

        # Build membership request for the group multicast address and local interface
        group = socket.inet_aton(self.ip)
        if self.interface_ip:
            try:
                local_ip = socket.inet_aton(self.interface_ip)
            except OSError as e:
                self.logger.warning(f"Invalid interface IP, using INADDR_ANY: {e}")
                local_ip = struct.pack("!I", socket.INADDR_ANY)
        else:
            local_ip = struct.pack("!I", socket.INADDR_ANY)

        mreq = group + local_ip
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)


        # Disable loopback (don't receive our own multicast packets)
        try:
            sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 0)
        except OSError as e:
            self.logger.warning(f"Could not disable loopback: {e}")

        # Set blocking or timeout mode
        if self.timeout == 0:
            sock.setblocking(False)
        else:
            sock.settimeout(self.timeout)

        self.sock = sock
        self.logger.info(f"Multicast client connected to {self.ip}:{self.port}")

    def receive(self):
        """Try to receive and decode one SSL_WrapperPacket. """

        # Ensure socket is connected
        if self.sock is None:
            raise RuntimeError("Client socket not connected. Call connect() first.")
        try:
            # Max packet size for SSL-Vision is safely under 4096 bytes; allocate a bit more than old 2048.
            data, _ = self.sock.recvfrom(MAX_PACKET_SIZE)
        except (BlockingIOError, socket.timeout):
            return None
        except OSError as e:
            self.logger.warning(f"Socket error: {e}")
            return None

        # Parse the protobuf message from the received data
        packet = SSL_WrapperPacket()
        try:
            packet.ParseFromString(data)
        except Exception as e:
            # If the packet is invalid, it is discarted.
            self.logger.debug(f"Invalid packet: {e}")
            return None
        return packet