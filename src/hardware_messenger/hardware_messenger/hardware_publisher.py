from rclpy.node import Node
import rclpy
import serial
import struct

from grsim_messenger.protobuf.grSim_Packet_pb2 import grSim_Packet
from grsim_messenger.protobuf.grSim_Commands_pb2 import grSim_Robot_Command

from system_interfaces.msg import TeamCommand
from grsim_messenger.inverse_kinematics import apply_inverse_kinematics


class HardwarePublisher(Node):
    def __init__(self) -> None:
        super().__init__("hardware_publisher")

        # Parameters settings.
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 115200)

        self.port = self.get_parameter("port").get_parameter_value().string_value
        self.baudrate = (
            self.get_parameter("baudrate").get_parameter_value().integer_value
        )

        self.serial_writer = serial.Serial(self.port, self.baudrate)
        self.serial_writer.open()

        self.commandSubscriber = self.create_subscription(
            TeamCommand, "CommandTopic", self.translate_command, 10
        )

        self.timer = self._node.create_timer(
            1 / 120, self.publish
        )  # publish to serial at 120Hz

    def publish(self):
        self.serial_writer.write(struct.pack(self.format_string, *self._command))

    # TODO: Add address
    def translate_command(self, command):
        self._command = [0 for _ in range(5 * len(command.robots))]

        # format string tells struct how to pack the data
        # x: Pad byte (no data)
        # c: char (1 byte)
        # b: signed char (1 byte)
        # B: unsigned char (1 byte)
        # ?: _Bool (1 byte)
        # h: short (2 bytes)
        # H: unsigned short (2 bytes)
        # i: int (4 bytes)
        # I: unsigned int (4 bytes)
        # l: long (4 bytes)
        # L: unsigned long (4 bytes)
        # q: long long (8 bytes)
        # Q: unsigned long long (8 bytes)
        # f: float (4 bytes)
        # d: double (8 bytes)
        # s: char[] (string)
        # p: char[] (Pascal string)
        # P: void * (pointer)
        self.format_string = "Ifff?" * len(command.robots)

        for robot in command.robots:
            offset = robot.robot_id * 5
            self._command[0 + offset] = int(robot.robot_id)
            self._command[1 + offset] = float(robot.linear_velocity_x)
            self._command[2 + offset] = float(robot.linear_velocity_y)
            self._command[3 + offset] = float(robot.angular_velocity)
            self._command[4 + offset] = int(robot.kick)


def main(args=None):
    rclpy.init(args=args)
    node = HardwarePublisher()

    rclpy.spin(node)


if __name__ == "__main__":
    main()
