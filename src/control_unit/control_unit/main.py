import rclpy
from rclpy.executors import MultiThreadedExecutor

from control_unit.game_watcher import GameWatcher
from control_unit.coach import Coach
from control_unit.command_publisher import CommandPublisher

def main():
    rclpy.init()
    
    #Reads all the topics and updates the blackboard
    game_watcher = GameWatcher()
    
    #Update blackboard for coach creation
    rclpy.spin_once(game_watcher, timeout_sec=0.2)
    
    #If there is no GUI, the default max number of robots is 3
    if game_watcher.blackboard is None:
        max_robots = 3
    else:
        max_robots = game_watcher.blackboard.gui.max_robots
    
    #num_threads is 6 because:
    #   1 for game_watcher
    #   1 for coach
    #   1 for command_sender
    #   max_robots for robots
    main_executor = MultiThreadedExecutor(num_threads=(3 + max_robots))

    #Has strategy for macro plays, has all ally robots 
    coach = Coach(main_executor, None)
    
    command_publisher = CommandPublisher(coach)
    
    main_executor.add_node(game_watcher)
    main_executor.add_node(coach)
    main_executor.add_node(command_publisher)
    
    main_executor.spin()
    
if __name__ == '__main__':
    main()