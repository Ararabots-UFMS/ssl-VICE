import rclpy
from rclpy.executors import MultiThreadedExecutor

from strategy.skills.move_to import MoveTo
from strategy.skills.robot_client import RobotClient


def alternate_command(args=None):
    rclpy.init(args=args)

    robot_cli = RobotClient(tick_period=0.1)

    waypoints = {
        0: [(0.0, 0.0), (1200.0, 1000.0), (-1200.0, 1000.0)],
        1: [(1200.0, 1000.0), (-1200.0, 1000.0), (0.0, 0.0)],
        2: [(-1200.0, 1000.0), (0.0, 0.0), (1200.0, 1000.0)],
    }

    indices = {rid: 0 for rid in waypoints.keys()}

    def cycle_skills():
        for rid, pts in waypoints.items():
            last_ok = robot_cli.last_response_success.get(rid, None)
            if last_ok:
                indices[rid] = (indices[rid] + 1) % len(pts)
                x, y = pts[indices[rid]]
                skill = MoveTo(
                    f"move_{rid}_{indices[rid]}", rid, target_x=x, target_y=y
                )
                robot_cli.set_skill(skill)

    robot_cli.create_timer(4.0, cycle_skills)

    for rid, pts in waypoints.items():
        x, y = pts[0]
        robot_cli.set_skill(MoveTo(f"init_{rid}", rid, target_x=x, target_y=y))

    executor = MultiThreadedExecutor()
    executor.add_node(robot_cli)

    try:
        executor.spin()
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
        MoveTo(name="tri_r0", robot_id=0, target_x=-100.0, target_y=0.0),
        MoveTo(name="tri_r1", robot_id=1, target_x=1200.0, target_y=1000.0),
        MoveTo(name="tri_r2", robot_id=2, target_x=-1200.0, target_y=1000.0),
    ]

    for cmd in cmds:
        robot_cli.set_skill(cmd)

    rclpy.spin(robot_cli)
    rclpy.shutdown()


if __name__ == "__main__":
    # alternate_command()
    unique_command()
