import rclpy
from rclpy.executors import MultiThreadedExecutor

from control_unit.game_watcher import GameWatcher
from control_unit.coach import Coach

def main():
    rclpy.init()
    
    #num_threads is 8 because:
    #   1 for game_watcher
    #   3 for topic subscribers
    #   1 for coach
    #   3 for robots
    main_executor = MultiThreadedExecutor(num_threads=8)

    #Reads all the topics and updates the blackboard
    game_watcher = GameWatcher(main_executor)
    
    #Update blackboard for coach creation
    rclpy.spin_once(game_watcher, timeout_sec=0.2)
    
    #Has strategy for macro plays, has all ally robots 
    coach = Coach(main_executor, None)    
    
    main_executor.add_node(game_watcher)
    main_executor.add_node(coach)
    
    main_executor.spin()
    
if __name__ == '__main__':
    main()