from rclpy.node import Node
import rclpy
import serial

from system_interfaces.msg import TeamCommand


class HardwarePublisher(Node):
    def __init__(self) -> None:
        super().__init__("hardware_publisher")

        # Parameters settings.
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 230400)

        self.port = self.get_parameter("port").get_parameter_value().string_value
        self.baudrate = (
            self.get_parameter("baudrate").get_parameter_value().integer_value
        )

        self._command = bytearray()

        self.serial_writer = serial.Serial(self.port, self.baudrate, timeout=0.1)

        self.commandSubscriber = self.create_subscription(
            TeamCommand, "commandTopic", self.translate_command, 10
        )

        self.timer = self.create_timer(
            1 / 120, self.publish_command
        )  # publish to serial at 120Hz

    def encode_command(self, robot_id, vx, vy, angular_speed, angle, kick_front):
        """ Encode command in binary format compatible with RobotCommand.decodeCommand() """
        # Pack the data into 11 bytes as expected by decodeCommand
        bytes_data = bytearray(11)
        
        # Byte 0: (id << 4) | (vx_linear >> 16)
        bytes_data[0] = (robot_id << 4) | ((vx & 0xF0000) >> 16)
        
        # Byte 1, 2: Rest of vx_linear
        bytes_data[1] = (vx >> 8) & 0xFF
        bytes_data[2] = vx & 0xFF
        
        # Byte 3, 4, 5(high nibble): vy_linear
        bytes_data[3] = (vy >> 12) & 0xFF
        bytes_data[4] = (vy >> 4) & 0xFF
        bytes_data[5] = (vy & 0x0F) << 4
        
        # Byte 5(low nibble), 6, 7: angular_speed
        bytes_data[5] |= (angular_speed >> 16) & 0x0F
        bytes_data[6] = (angular_speed >> 8) & 0xFF
        bytes_data[7] = angular_speed & 0xFF
        
        # Byte 8, 9, 10(high nibble): angle
        bytes_data[8] = (angle >> 12) & 0xFF
        bytes_data[9] = (angle >> 4) & 0xFF
        bytes_data[10] = (angle & 0x0F) << 4
        
        # Byte 10(bit 3): kick_front
        if kick_front:
            bytes_data[10] |= 0x08
        
        return bytes_data

    def publish_command(self):
        if len(self._command) > 0:
            self.serial_writer.write(self._command)

    def translate_command(self, command):
        self._command = bytearray()

        for robot in command.robots:
            robot_command = self.encode_command(
                robot.robot_id,
                int(robot.linear_velocity_x * 1000),
                int(-robot.linear_velocity_y * 1000),
                int(-robot.angular_velocity * 1000),
                int(robot.orientation * 1000),
                int(robot.kick),
            )

            self._command.extend(robot_command)


def main(args=None):
    rclpy.init(args=args)
    node = HardwarePublisher()

    rclpy.spin(node)


if __name__ == "__main__":
    main()
