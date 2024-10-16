
import socket
import grsim_messenger.protobuf.grSim_Packet_pb2 as grSim_Packet_pb2


class grSimSender:
    
    def __init__(self, ip:str, port:int):
        """Client that connects and receives messages from ssl-vision"""
        
        self.ip = ip
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 32)

    def send(self, message:grSim_Packet_pb2.grSim_Packet):
        """Encode and send message."""
        msg = message.SerializeToString()
        self.sock.sendto(msg, (self.ip, self.port))
        