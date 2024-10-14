from rclpy.node import Node
import rclpy
import serial
import struct

from grsim_messenger.protobuf.grSim_Packet_pb2 import grSim_Packet
from grsim_messenger.protobuf.grSim_Commands_pb2 import grSim_Robot_Command

from system_interfaces.msg import TeamCommand, GUIMessage
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

        # Format string tells struct how to pack the data
        # B: unsigned char (1 byte)
        # f: float (4 bytes)
        header_kicker_format = "B"
        address_format = "BBBBB"
        velocities_format = "fff"
        self.robot_command_format = (
            address_format + header_kicker_format + velocities_format
        )

        # Commands
        # 128: only velocities and kicker
        # 64:
        # 32:
        # 16:
        # 4:
        # 2:
        self.command_headers = [128]

        self.header_chooser = -1

        self.serial_writer = serial.Serial(self.port, self.baudrate)
        self.serial_writer.open()

        self.commandSubscriber = self.create_subscription(
            TeamCommand, "CommandTopic", self.translate_command, 10
        )

        self.gui_subscriber = self.create_subscription(
            GUIMessage, "guiTopic", self.gui_callback, 10
        )

        self.timer = self._node.create_timer(
            1 / 120, self.publish
        )  # publish to serial at 120Hz

    def publish(self):
        self.serial_writer.write(struct.pack(self.format_string, *self._command))

    def gui_callback(self, msg):
        # self.robot_addresses = msg.addresses
        pass

    # TODO: Add address
    def translate_command(self, command):
        self._command = []

        self.format_string = self.robot_command_format * len(command.robots)

        self.header_chooser += 1
        if self.header_chooser > len(self.command_headers):
            self.header_chooser = 0

        for robot in command.robots:
            robot_command = [
                self.robot_addresses[robot.id][0],
                self.robot_addresses[robot.id][1],
                self.robot_addresses[robot.id][2],
                self.robot_addresses[robot.id][3],
                self.robot_addresses[robot.id][4],
                self.command_headers[self.header_chooser] + int(robot.kicker),
                float(robot.linear_velocity_x),
                float(robot.linear_velocity_y),
                float(robot.angular_velocity),
            ]

            self._command += robot_command


def main(args=None):
    rclpy.init(args=args)
    node = HardwarePublisher()

    rclpy.spin(node)


if __name__ == "__main__":
    main()
