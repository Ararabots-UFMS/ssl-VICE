import simu_client
from protobuf.grSim_Packet_pb2 import grSim_Packet
from protobuf.grSim_Commands_pb2 import grSim_Robot_Command
# from .protobuf.grSim_Commands_pb2 import grSim_Commands
from protobuf.grSim_Replacement_pb2 import grSim_RobotReplacement
from protobuf.grSim_Replacement_pb2 import grSim_BallReplacement
from protobuf.grSim_Replacement_pb2 import grSim_Replacement

from copy import copy

import time
import numpy as np
from kino import *

from ruckig import InputParameter, OutputParameter, Result, Ruckig


def main(init_pos, goal_pos):
    client = simu_client.Client(ip='127.0.0.1', port=20011)
    packet = grSim_Packet()

    robot_replacement = grSim_RobotReplacement()
    robot_replacement.x = init_pos[0]
    robot_replacement.y = init_pos[1]
    robot_replacement.id = 0
    robot_replacement.dir = 0
    robot_replacement.yellowteam = 0

    packet.replacement.robots.append(robot_replacement)
    client.send(packet)

    time_unit = 0.01
    
    otg = Ruckig(3, 0.01)  # DoFs, control cycle
    inp = InputParameter(3)
    out = OutputParameter(3)

    # Set input parameters
    inp.current_position = [4.50, 3.0, 0.0]
    inp.current_velocity = [0.0, 0, 0]
    # inp.current_acceleration = [0.0, 0, 0]

    inp.target_position = [0, 0, 0]
    inp.target_velocity = [2, 0, 0]
    # inp.target_acceleration = [0.0, 0.0, 0.0]

    inp.max_velocity = [5.0, 3.0, 1.0]
    inp.max_acceleration = [1.0, 1.0, 1.0]
    # inp.max_jerk = [4.0, 3.0, 1.0]

    first_output, out_list = None, []
    res = Result.Working
    while res == Result.Working:
        res = otg.update(inp, out)

        out_list.append(copy(out))

        out.pass_to_input(inp)
        print(out)

        if not first_output:
            first_output = copy(out)

    for i, step in enumerate(out_list):
        start = time.time()
        packet = grSim_Packet()

        packet.commands.isteamyellow = False
        packet.commands.timestamp = 0

        
        offset = out_list[i-1].new_position[2] if i > 0 else 0
        a, b, c, d = apply_inverse_kino(step.new_velocity[0], step.new_velocity[1], step.new_velocity[2], offset)

        robot_command = grSim_Robot_Command()
        robot_command.id = 0
        robot_command.wheelsspeed = 1
        robot_command.velnormal = 0
        robot_command.kickspeedx = 0
        robot_command.kickspeedz = 0
        robot_command.veltangent = 0
        robot_command.velangular = 0
        robot_command.spinner = 0
        robot_command.wheel1 = a
        robot_command.wheel2 = b
        robot_command.wheel3 = c
        robot_command.wheel4 = d

        packet.commands.robot_commands.append(robot_command)

        client.send(packet)
        finish = time.time() - start
        time.sleep(time_unit - finish)
        # time.sleep(time_unit)

if __name__ == "__main__":
    init_x = float(input("Inital position X (mm)"))
    init_y = float(input("Inital position Y (mm)"))
    goal_x = float(input("Goal position X  (mm)"))
    goal_y = float(input("Goal position Y  (mm)"))
    main((init_x / 1000, init_y / 1000), (goal_x / 1000, goal_y / 1000))