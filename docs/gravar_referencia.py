#!/usr/bin/env python3
"""Record robot 1's tracker reference next to where it actually is, to a CSV.

Runs inside the vice container next to a campaign replay, and stops when killed:

    python3 gravar_referencia.py /tmp/referencia.csv &

Columns: wall time, referee command, ball, robot (vision), reference (tracker).
Used to tell a plan that swings sideways from a robot that does not follow its plan.
"""
import csv
import sys
import time

import rclpy
from rclpy.node import Node
from movement_interfaces.msg import TrajectoryPoint
from system_interfaces.msg import GameState

ROBO = 1


class Gravador(Node):
    def __init__(self, caminho):
        super().__init__("gravador_referencia")
        self.arquivo = open(caminho, "w", newline="")
        self.saida = csv.writer(self.arquivo)
        self.saida.writerow(["t", "cmd", "bx", "by", "rx", "ry", "rvx", "rvy", "rth",
                             "ref_x", "ref_y", "ref_vx", "ref_vy"])
        self.ref = None
        self.create_subscription(TrajectoryPoint, "movement_tracker/control_reference",
                                 self._referencia, 50)
        self.create_subscription(GameState, "game_state", self._estado, 50)

    def _referencia(self, msg):
        if msg.robot_id == ROBO:
            self.ref = (msg.pos.x, msg.pos.y, msg.vel.x, msg.vel.y)

    def _estado(self, msg):
        robo = next((r for r in msg.ally_robots if r.id == ROBO), None)
        if robo is None or not msg.balls or self.ref is None:
            return
        b = msg.balls[0]
        self.saida.writerow([round(time.time(), 3), msg.referee.command,
                             round(b.position_x), round(b.position_y),
                             round(robo.position_x, 1), round(robo.position_y, 1),
                             round(robo.velocity_x), round(robo.velocity_y),
                             round(robo.orientation, 3)] + [round(v, 1) for v in self.ref])


def main():
    rclpy.init()
    no = Gravador(sys.argv[1])
    try:
        rclpy.spin(no)
    except KeyboardInterrupt:
        pass
    finally:
        no.arquivo.close()


if __name__ == "__main__":
    main()
