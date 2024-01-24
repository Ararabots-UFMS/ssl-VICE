from system_interfaces.srv import PIDUpdate
import rclpy
from rclpy.node import Node


class PIDClientAsync(Node):

    def __init__(self, kp, ki, kd):
        super().__init__('pid_client_async')
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.pid_client = self.create_client(PIDUpdate, 'pid_update')
        while not self.pid_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = PIDUpdate.Request()

    def set_parameters(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def request_update(self, angle_diff):
        self.req.kp = self.kp
        self.req.ki = self.ki
        self.req.kd = self.kd
        self.req.angle = angle_diff
        print(f"Sending request to PIDUpdate service with kp={self.kp}, ki={self.ki}, kd={self.kd}, angle={angle_diff}")
        self.future = self.pid_client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    import sys

    rclpy.init(args=args)

    pid_client = PIDClientAsync(sys.argv[1], sys.argv[2], sys.argv[3])
    response = pid_client.send_request(int(sys.argv[4]))
    pid_client.get_logger().info(
        'Result of add_two_ints: for %d + %d + %d = %d' %
        (int(sys.argv[1]), int(sys.argv[2]), int(sys.argv[3]), response.sum))

    pid_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()