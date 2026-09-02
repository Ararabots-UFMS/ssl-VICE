import rclpy
from rclpy.node import Node
from flask import Flask
from flask_socketio import SocketIO
import threading

from utils.converter import todict

from movement_interfaces.msg import GUITrajectories
from system_interfaces.msg import VisionMessage, GUIMessage, GUIRobot, RefereeMessage
from system_interfaces.srv import ControlParams, SetKp, SetOrientation, StrategyCommand, UpdateObstacle
from std_srvs.srv import SetBool
from new_movement.entities.trajectory import Trajectory

app = Flask(__name__)
gui_socket = SocketIO(app, cors_allowed_origins="*")

VISION_NODE_NAME= "visionNode"
SIM_COMMUNICATION_NODE_NAME = "grsim_publisher"
REAL_COMMUNICATION_NODE_NAME = "hardware_publisher"
REFEREE_NODE_NAME = "refereeNode"

class APINode(Node):
    def __init__(self, name):
        super().__init__(name)

        self.publisher = self.create_publisher(GUIMessage, "guiTopic", 10)

        self.vision_running = False
        self.vision_subscriber = None

        self.trajectory_subscriber = self.create_subscription(
            GUITrajectories, "gui/trajectories", self.emit_trajectory_message, 10
        )
        self.referee_subscriber = self.create_subscription(
            RefereeMessage, "refereeTopic", self.emit_referee_message, 10
        )

        self.communication_running = False

        self.referee_running = False

        self.robots = []
        self.robot_count = 0

        self.is_field_side_right = True
        self.is_team_color_yellow = True
        self.is_play_pressed = False
        self.is_simulation = True

        self.strategy_client = self.create_client(StrategyCommand, "strategy_command")
        self.pid_client = self.create_client(ControlParams, "update_pid")
        self.kp_angular_client = self.create_client(SetKp, "update_kp_angular")
        self.set_orientation_client = self.create_client(SetOrientation, "set_orientation")
        self.update_obstacles_client = self.create_client(UpdateObstacle, "update_obstacles")
        self.set_team_color_client = self.create_client(SetBool, "set_team_color")

        self.get_logger().info("API Node started")

        self.create_timer(0.5, self.publish_gui_data)
        self.create_timer(2.0, self.check_strategy_services_status)

    def handle_connect(self):
        self.get_logger().info("Client connected")
        self.vision_subscriber = self.create_subscription(
            VisionMessage, "visionTopic", self.emit_vision_message, 10
        )
    
    def emit_vision_message(self, msg: VisionMessage) -> None:
        data = todict(msg)

        gui_socket.emit("vision_update", {
            "yellow": data["yellow_robots"],
            "blue":   data["blue_robots"],
            "balls":  data["balls"],
        })

    def emit_trajectory_message(self, msg: GUITrajectories) -> None:
        trajectories_data = []

        for idx, cur_trajectory_msg in enumerate(msg.current_trajectories):
            if not cur_trajectory_msg.segments:
                continue

            cur_trajectory = Trajectory.from_msg(cur_trajectory_msg)
            time_offset = msg.time_offsets[idx]
            total_duration = cur_trajectory.get_total_duration()

            # time_offset → end
            current_points = []
            if time_offset < total_duration:
                states = cur_trajectory.to_list(
                    time_step=0.2,
                    start_time=time_offset,
                    end_time=total_duration,
                    output_states=True,
                )
                current_points = [
                    {
                        "x": s.position.x,
                        "y": s.position.y,
                        "vx": s.velocity.x,
                        "vy": s.velocity.y,
                    }
                    for s in states
                ]

            # Pending trajectory t=0 → end
            pending_points = []
            pending_handoff = None
            pending_msg = msg.pending_trajectories[idx]
            if pending_msg.segments:
                pending_trajectory = Trajectory.from_msg(pending_msg)
                pending_duration = pending_trajectory.get_total_duration()
                if pending_duration > 1e-3:
                    states = pending_trajectory.to_list(
                        time_step=0.2,
                        start_time=0.0,
                        end_time=pending_duration,
                        output_states=True,
                    )
                    pending_points = [
                        {
                            "x": s.position.x,
                            "y": s.position.y,
                            "vx": s.velocity.x,
                            "vy": s.velocity.y,
                        }
                        for s in states
                    ]
                    pending_handoff = pending_msg.handoff_stamp

            trajectories_data.append({
                "robot_id": cur_trajectory_msg.robot_id,
                "current": {
                    "points": current_points,
                    "total_duration": total_duration,
                    "time_offset": time_offset,
                },
                "pending": {
                    "points": pending_points,
                    "handoff_stamp": pending_handoff,
                },
            })

        gui_socket.emit("trajectory_update", {"trajectories": trajectories_data})

    def emit_referee_message(self, msg: RefereeMessage) -> None:
        """Forward referee messages to the GUI."""
        data = todict(msg)

        # try:
            # cmd = data.get("command")
            # counter = data.get("command_counter")
            # self.get_logger().info(f"emit_referee_message: command={cmd} counter={counter}")
        # except Exception:
            # pass

        payload = {
            "stage": data.get("stage"),
            "stage_time_left": data.get("stage_time_left"),
            "command": data.get("command"),
            "command_counter": data.get("command_counter"),
            "current_action_time_remaining": data.get("current_action_time_remaining"),
            "teams": data.get("teams", []),
        }

        gui_socket.emit("referee_update", payload)

    def handle_disconnect(self):
        self.get_logger().info("Client disconneted")
        self.vision_subscriber = None
        self.trajectory_subscriber = None
        self.referee_subscriber = None

    def handle_field_side(self, is_field_side_right):
        is_field_side_right
    
        self.get_logger().info(f"Is team field side left? {is_field_side_right}")

        self.is_field_side_right = is_field_side_right

    def handle_team_color(self, is_team_color_yellow):

        self.get_logger().info(f"Is team color blue? {is_team_color_yellow}")

        self.is_team_color_yellow = is_team_color_yellow

    def handle_simulation(self, is_simulation):
        self.get_logger().info(f"Is sumulation? {is_simulation}")
        self.is_simulation = is_simulation

    def update_vision_status(self, nodes_running):
        is_vision_running = VISION_NODE_NAME in nodes_running

        if is_vision_running == self.vision_running:
            return

        self.vision_running = not self.vision_running
        gui_socket.emit("visionStatus", {"status": self.vision_running})

    def update_communication_status(self, nodes_running):
        communication_node_name = SIM_COMMUNICATION_NODE_NAME if self.is_simulation else REAL_COMMUNICATION_NODE_NAME
        is_communication_running = communication_node_name in nodes_running

        if is_communication_running == self.communication_running:
            return

        self.communication_running = not self.communication_running
        gui_socket.emit("communicationStatus", {"status": self.communication_running})

    def update_referee_status(self, nodes_running):
        referee_node_name = REFEREE_NODE_NAME
        is_referee_running = referee_node_name in nodes_running

        if is_referee_running == self.referee_running:
            return

        self.referee_running = not self.referee_running
        gui_socket.emit("refereeStatus", {"status": self.referee_running})

    def create_message(self) -> GUIMessage:
        msg = GUIMessage()
        msg.is_field_side_left = not self.is_field_side_right
        msg.is_team_color_yellow = self.is_team_color_yellow
        msg.is_play_pressed = self.is_play_pressed
        for gui_robot in self.robots:
            robot = GUIRobot()
            robot.id = gui_robot["id"]
            robot.name = gui_robot["name"]
            robot.address = [int(i) for i in gui_robot["address"].split(",")]
            robot.kp = float(gui_robot["kp"])
            robot.ki = float(gui_robot["ki"])
            robot.kd = float(gui_robot["kd"])

            msg.robots.append(robot)
        msg.robot_count = self.robot_count
        return msg

    def publish_gui_data(self) -> None:
        message = self.create_message()
        self.publisher.publish(message)
        nodes_running = self.get_node_names()
        self.update_vision_status(nodes_running)
        self.update_communication_status(nodes_running)
        self.update_referee_status(nodes_running)

    def handle_config_button(self, msg):
        self.get_logger().info("Configuration saved")
        self.robot_count = len(msg)
        self.robots = msg
        self.publish_gui_data()

    # Strategy command handlers
    def handle_strategy_command(self, data):
        """Handle strategy command from web GUI"""
        if not self.strategy_client.wait_for_service(timeout_sec=1.0):
            gui_socket.emit("strategy_response", {
                "success": False, 
                "message": "Strategy command service is not available"
            })
            return

        try:
            request = StrategyCommand.Request()
            request.id = int(data['robot_id'])
            request.position_x = float(data['position_x'])
            request.position_y = float(data['position_y'])
            request.velocity_x = float(data.get('velocity_x', 0.0))
            request.velocity_y = float(data.get('velocity_y', 0.0))

            self.get_logger().info(
                f"Sending strategy command: ID={request.id}, "
                f"Pos=({request.position_x}, {request.position_y}), "
                f"Vel=({request.velocity_x}, {request.velocity_y})"
            )

            future = self.strategy_client.call_async(request)
            future.add_done_callback(self.handle_strategy_response)

        except Exception as e:
            self.get_logger().error(f"Failed to send strategy command: {e}")
            gui_socket.emit("strategy_response", {
                "success": False,
                "message": f"Failed to send command: {e}"
            })

    def handle_strategy_response(self, future):
        """Handle strategy command response"""
        try:
            response = future.result()
            gui_socket.emit("strategy_response", {
                "success": response.success,
                "message": "Command executed successfully" if response.success else "Command failed"
            })
        except Exception as e:
            self.get_logger().error(f"Strategy command failed: {e}")
            gui_socket.emit("strategy_response", {
                "success": False,
                "message": f"Service call failed: {e}"
            })

    def handle_update_pid(self, data):
        """Handle PID update from web GUI"""
        if not self.pid_client.wait_for_service(timeout_sec=1.0):
            gui_socket.emit("pid_response", {
                "success": False,
                "message": "PID service is not available"
            })
            return

        try:
            request = ControlParams.Request()
            request.id = int(data['robot_id'])
            request.kp = float(data['kp'])
            request.ki = float(data['ki'])
            request.kd = float(data['kd'])

            self.get_logger().info(
                f"Updating PID for robot {request.id}: "
                f"Kp={request.kp}, Ki={request.ki}, Kd={request.kd}"
            )

            future = self.pid_client.call_async(request)
            future.add_done_callback(self.handle_pid_response)

        except Exception as e:
            self.get_logger().error(f"Failed to update PID: {e}")
            gui_socket.emit("pid_response", {
                "success": False,
                "message": f"Failed to update PID: {e}"
            })

    def handle_pid_response(self, future):
        """Handle PID update response"""
        try:
            response = future.result()
            gui_socket.emit("pid_response", {
                "success": response.success,
                "message": "PID updated successfully" if response.success else "PID update failed"
            })
        except Exception as e:
            self.get_logger().error(f"PID update failed: {e}")
            gui_socket.emit("pid_response", {
                "success": False,
                "message": f"PID service call failed: {e}"
            })

    def handle_update_kp_angular(self, data):
        """Handle Kp angular update from web GUI"""
        if not self.kp_angular_client.wait_for_service(timeout_sec=1.0):
            gui_socket.emit("kp_angular_response", {
                "success": False,
                "message": "Kp angular service is not available"
            })
            return

        try:
            request = SetKp.Request()
            request.kp = float(data['kp'])

            self.get_logger().info(f"Updating Kp angular: {request.kp}")

            future = self.kp_angular_client.call_async(request)
            future.add_done_callback(self.handle_kp_angular_response)

        except Exception as e:
            self.get_logger().error(f"Failed to update Kp angular: {e}")
            gui_socket.emit("kp_angular_response", {
                "success": False,
                "message": f"Failed to update Kp angular: {e}"
            })

    def handle_kp_angular_response(self, future):
        """Handle Kp angular update response"""
        try:
            response = future.result()
            gui_socket.emit("kp_angular_response", {
                "success": response.success,
                "message": "Kp angular updated successfully" if response.success else "Kp angular update failed"
            })
        except Exception as e:
            self.get_logger().error(f"Kp angular update failed: {e}")
            gui_socket.emit("kp_angular_response", {
                "success": False,
                "message": f"Kp angular service call failed: {e}"
            })

    def handle_set_orientation(self, data):
        """Handle set orientation from web GUI"""
        if not self.set_orientation_client.wait_for_service(timeout_sec=1.0):
            gui_socket.emit("orientation_response", {
                "success": False,
                "message": "Set orientation service is not available"
            })
            return

        try:
            request = SetOrientation.Request()
            request.robot_id = int(data['robot_id'])
            request.orientation = float(data['orientation'])

            self.get_logger().info(
                f"Setting orientation for robot {request.robot_id}: {request.orientation} rad"
            )

            future = self.set_orientation_client.call_async(request)
            future.add_done_callback(self.handle_orientation_response)

        except Exception as e:
            self.get_logger().error(f"Failed to set orientation: {e}")
            gui_socket.emit("orientation_response", {
                "success": False,
                "message": f"Failed to set orientation: {e}"
            })

    def handle_orientation_response(self, future):
        """Handle set orientation response"""
        try:
            response = future.result()
            gui_socket.emit("orientation_response", {
                "success": response.success,
                "message": "Orientation set successfully" if response.success else "Set orientation failed"
            })
        except Exception as e:
            self.get_logger().error(f"Set orientation failed: {e}")
            gui_socket.emit("orientation_response", {
                "success": False,
                "message": f"Set orientation service call failed: {e}"
            })

    def handle_update_obstacles(self, data):
        """Handle update obstacles from web GUI"""
        if not self.update_obstacles_client.wait_for_service(timeout_sec=1.0):
            gui_socket.emit("obstacles_response", {
                "success": False,
                "message": "Update obstacles service is not available"
            })
            return

        try:
            request = UpdateObstacle.Request()
            request.id = int(data['robot_id'])
            request.field_border = bool(data.get('field_border', False))
            request.penalty_area = bool(data.get('penalty_area', False))
            request.center_area = bool(data.get('center_area', False))
            request.ball = bool(data.get('ball', False))
            
            # Parse enemy and ally IDs
            enemy_ids = data.get('enemy_ids', [])
            ally_ids = data.get('ally_ids', [])
            
            if isinstance(enemy_ids, str):
                enemy_ids = [int(x.strip()) for x in enemy_ids.split(',') if x.strip()]
            request.enemy_ids = enemy_ids
            
            if isinstance(ally_ids, str):
                ally_ids = [int(x.strip()) for x in ally_ids.split(',') if x.strip()]
            request.ally_ids = ally_ids

            self.get_logger().info(
                f"Updating obstacles for robot {request.id}: "
                f"field_border={request.field_border}, penalty_area={request.penalty_area}, "
                f"center_area={request.center_area}, ball={request.ball}, "
                f"enemy_ids={request.enemy_ids}, ally_ids={request.ally_ids}"
            )

            future = self.update_obstacles_client.call_async(request)
            future.add_done_callback(self.handle_obstacles_response)

        except Exception as e:
            self.get_logger().error(f"Failed to update obstacles: {e}")
            gui_socket.emit("obstacles_response", {
                "success": False,
                "message": f"Failed to update obstacles: {e}"
            })

    def handle_obstacles_response(self, future):
        """Handle update obstacles response"""
        try:
            response = future.result()
            gui_socket.emit("obstacles_response", {
                "success": response.success,
                "message": "Obstacles updated successfully" if response.success else "Update obstacles failed"
            })
        except Exception as e:
            self.get_logger().error(f"Update obstacles failed: {e}")
            gui_socket.emit("obstacles_response", {
                "success": False,
                "message": f"Update obstacles service call failed: {e}"
            })

    def handle_team_color_service(self, data):
        """Handle team color change service call"""
        if not self.set_team_color_client.wait_for_service(timeout_sec=1.0):
            gui_socket.emit("team_color_response", {
                "success": False,
                "message": "Set team color service is not available"
            })
            return

        try:
            request = SetBool.Request()
            request.data = bool(data.get('is_yellow', False))

            self.get_logger().info(f"Setting team color to: {'Yellow' if request.data else 'Blue'}")

            future = self.set_team_color_client.call_async(request)
            future.add_done_callback(self.handle_team_color_service_response)

        except Exception as e:
            self.get_logger().error(f"Failed to set team color: {e}")
            gui_socket.emit("team_color_response", {
                "success": False,
                "message": f"Failed to set team color: {e}"
            })

    def handle_team_color_service_response(self, future):
        """Handle team color service response"""
        try:
            response = future.result()
            gui_socket.emit("team_color_response", {
                "success": response.success,
                "message": "Team color set successfully" if response.success else "Set team color failed"
            })
        except Exception as e:
            self.get_logger().error(f"Set team color failed: {e}")
            gui_socket.emit("team_color_response", {
                "success": False,
                "message": f"Set team color service call failed: {e}"
            })

    def check_strategy_services_status(self):
        """Check status of all strategy services and emit to GUI"""
        services_status = {
            "strategy": self.strategy_client.service_is_ready(),
            "pid": self.pid_client.service_is_ready(),
            "kp_angular": self.kp_angular_client.service_is_ready(),
            "orientation": self.set_orientation_client.service_is_ready(),
            "obstacles": self.update_obstacles_client.service_is_ready(),
            "team_color": self.set_team_color_client.service_is_ready()
        }

        gui_socket.emit("services_status", services_status)
        return services_status

def run_socket():
    gui_socket.run(app, allow_unsafe_werkzeug=True)

def main(args=None):
    rclpy.init(args=args)

    node = APINode("api_node")
    gui_socket.on_event("connect", node.handle_connect, namespace="")
    gui_socket.on_event("disconnect", node.handle_disconnect, namespace="")
    gui_socket.on_event("fieldSide", node.handle_field_side, namespace="")
    gui_socket.on_event("teamColor", node.handle_team_color, namespace="")
    gui_socket.on_event("fieldMode", node.handle_simulation, namespace="")
    gui_socket.on_event("configSaveButton", node.handle_config_button, namespace="")
    
    # Strategy command events
    gui_socket.on_event("strategyCommand", node.handle_strategy_command, namespace="")
    gui_socket.on_event("updatePID", node.handle_update_pid, namespace="")
    gui_socket.on_event("updateKpAngular", node.handle_update_kp_angular, namespace="")
    gui_socket.on_event("setOrientation", node.handle_set_orientation, namespace="")
    gui_socket.on_event("updateObstacles", node.handle_update_obstacles, namespace="")
    gui_socket.on_event("setTeamColorService", node.handle_team_color_service, namespace="")
    gui_socket.on_event("checkServicesStatus", node.check_strategy_services_status, namespace="")
    try:
        socket_thread = threading.Thread(target=run_socket, daemon=True)
        socket_thread.start()
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
