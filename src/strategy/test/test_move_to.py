import rclpy
from rclpy.executors import MultiThreadedExecutor

from strategy.skills.move import MoveSkill
from strategy.robot_client import RobotClient
import time

def alternate_command(args=None):
    rclpy.init(args=args)

    robot_cli = RobotClient()
    executor = MultiThreadedExecutor()
    executor.add_node(robot_cli)

    # três vértices do triângulo
    triangle = [(0.0, 0.0), (1200.0, 1000.0), (-1200.0, 1000.0)]

    # cada robô começa em um vértice diferente
    waypoints = {rid: triangle[:] for rid in range(3)}
    indices = {0: 0, 1: 1, 2: 2}

    try:
        while rclpy.ok():
            for rid in waypoints:
                idx = indices[rid]
                x, y = waypoints[rid][idx]
                cmd = MoveSkill(
                    name=f"move_r{rid}_to_{x}_{y}",
                    robot_id=rid,
                    target_x=x,
                    target_y=y,
                )
                robot_cli.send_request(cmd)
                indices[rid] = (idx + 1) % len(triangle)
            time.sleep(4)
    except KeyboardInterrupt:
        pass
    finally:
        executor.remove_node(robot_cli)
        robot_cli.destroy_node()
        rclpy.shutdown()



def unique_command(args=None):
    rclpy.init(args=args)

    robot_cli = RobotClient()

    cmds = [
        MoveSkill(name="tri_r0", robot_id=0, target_x=0.0, target_y=0.0),
        MoveSkill(name="tri_r1", robot_id=2, target_x=1200.0, target_y=1000.0),
        MoveSkill(name="tri_r2", robot_id=1, target_x=-1200.0, target_y=1000.0),
    ]

    for cmd in cmds:
        robot_cli.send_request(cmd)

    rclpy.spin(robot_cli)
    rclpy.shutdown()


if __name__ == "__main__":
    alternate_command()
    #unique_command()
