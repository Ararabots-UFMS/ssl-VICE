import rclpy
from rclpy.executors import MultiThreadedExecutor

from control_unit.game_watcher import GameWatcher
from vision.vision_node import Vision
from new_movement.driver import PathDriver
from control.control import Controller
from grsim_messenger.grsim_publisher import grSimPublisher
# from control_unit.coach import Coach
# from control_unit.command_publisher import CommandPublisher


def main():
    rclpy.init()

    # Reads all the topics and updates the blackboard
    game_watcher = GameWatcher()
    driver = PathDriver()
    control = Controller()
    # gui = StrategyCommandGUI()

    # #If there is no GUI, the default max number of robots is 3
    # if game_watcher.blackboard is None:
    max_robots = 3
    # else:
    #     max_robots = game_watcher.blackboard.gui.max_robots

    # num_threads is 6 because:
    #   1 for game_watcher
    #   1 for coach
    #   1 for command_sender
    #   max_robots for robots
    main_executor = MultiThreadedExecutor(num_threads=(3 + max_robots))

    main_executor.add_node(game_watcher)
    main_executor.add_node(driver)
    main_executor.add_node(control)
    # main_executor.add_node(gui)

    main_executor.spin()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
