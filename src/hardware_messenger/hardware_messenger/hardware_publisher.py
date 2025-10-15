from rclpy.node import Node
import rclpy
import serial
import struct

from system_interfaces.msg import TeamCommand, GUIMessage


class HardwarePublisher(Node):
    def __init__(self) -> None:
        super().__init__("hardware_publisher")

        # Parameters settings.
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 1000000)

        self.port = self.get_parameter("port").get_parameter_value().string_value
        self.baudrate = (
            self.get_parameter("baudrate").get_parameter_value().integer_value
        )

        # Format string tells struct how to pack the data
        # B: unsigned char (1 byte)
        # f: float (4 bytes)
        name = "B"
        velocities_format = "fff"
        kick = "B"
        self.robot_command_format = name + velocities_format + kick

        self.robots = {}

        self._command = []
        self.robot_count = 0
        self.first_message_sent = False

        self.serial_writer = serial.Serial(self.port, self.baudrate, timeout=0.1)

        self.commandSubscriber = self.create_subscription(
            TeamCommand, "commandTopic", self.translate_command, 10
        )

        self.guiSubscriber = self.create_subscription(
            GUIMessage, "guiTopic", self.gui_callback, 10
        )

        self.timer = self.create_timer(
            1 / 60, self.publish_command
        )  # publish to serial at 60Hz

    def publish_command(self):
        self.get_logger().info(
            f"first message sent: {self.first_message_sent}, robot count: {self.robot_count}"
        )
        if len(self._command) == 0:
            self.get_logger().info(f"Returning")
            return
        elif not self.first_message_sent and self.robot_count != 0:
            self.get_logger().info(f"Sending robot count {self.robot_count}")
            self.serial_writer.write(struct.pack("B", *[self.robot_count]))
            self.first_message_sent = True
        elif self.first_message_sent and self.robot_count != 0:
            # while self.serial_writer.in_waiting == 0:
            #     pass
            if self.serial_writer.in_waiting != 0:
                lido = self.serial_writer.read()
                self.get_logger().info(f"lido: {lido}")
                if lido == b"R":
                    self.get_logger().info(f"command: {self._command}")
                    self.serial_writer.write(
                        struct.pack(self.format_string, *self._command)
                    )
                    self.serial_writer.reset_input_buffer()
                    self.get_logger().info(f"Sent command")

    def gui_callback(self, message):
        self.robot_count = message.robot_count

        for robot in message.robots:
            self.robots[robot.id] = robot

    def translate_command(self, command):
        self._command = []

        if self.robot_count == 0:
            return

        self.format_string = "=" + self.robot_command_format * self.robot_count

        for robot in command.robots:
            robot_command = [
                ord(self.robots[robot.robot_id].name[0]),
                float(robot.linear_velocity_x),
                float(robot.linear_velocity_y),
                float(robot.angular_velocity),
                int(robot.kick),
            ]

            self._command.extend(robot_command)


def main(args=None):
    rclpy.init(args=args)
    node = HardwarePublisher()

    rclpy.spin(node)


if __name__ == "__main__":
    main()
