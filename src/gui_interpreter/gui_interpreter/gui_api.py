from flask import Flask
from flask_socketio import SocketIO, emit
from threading import Thread
import subprocess

from gui_interpreter.gui_publisher import GUIPublisher
from utils.vision_subscriber import VisionSubscriber
from utils.converter import todict
import rclpy

app = Flask(__name__)
gui_socket = SocketIO(app, cors_allowed_origins="*")
rclpy.init()
gui_publisher = GUIPublisher()
global vision_running

############# CONNECT TO GUI #############


@gui_socket.on("connect")
def handle_connect():
    print("Client connected")
    global thread
    global stop_thread
    stop_thread = False
    if thread is None:
        thread = Thread(target=vision_thread)
        thread.start()


############# RECEIVE VISION INTERPRETER DATA #############


def vision_thread():
    vision_subs = VisionSubscriber()
    while True:
        #TODO: check if spin is better than spin_once
        rclpy.spin_once(vision_subs)
        data = vision_subs.get_data()
        data = todict(data)
        gui_socket.emit("vision_msg", {"data": data})


############# DISCONECT FROM GUI #############


@gui_socket.on("disconnect")
def handle_disconnect():
    print("Client disconnected")
    global thread
    global stop_thread
    stop_thread = True
    thread = None


############# FIELD SIDE BUTTON #############


@gui_socket.on("fieldSide")
def handle_field_side(is_field_side_left):
    # if is_field_side_left:
    #     print("Team field side is left", flush=True)
    # else:
    #     print("Team field side is left", flush=True)
    # print(is_field_side_left)
    gui_publisher.is_field_side_left = is_field_side_left
    gui_publisher.publish_gui_data()


############# TEAM COLOR BUTTON #############


@gui_socket.on("teamColor")
def handle_team_color(is_team_color_blue):
    # if is_team_color_blue:
    #     print("Team color is blue", flush=True)
    # else:
    #     print("Team color is yellow", flush=True)
    gui_publisher.is_team_color_blue = is_team_color_blue
    gui_publisher.publish_gui_data()


############# OUTPUT HANDLER #############


def handle_output(pipe, callback):
    """
    Read output from a pipe and pass it to a callback line by line.
    """
    with pipe:
        for line in iter(pipe.readline, b""):
            callback(line.decode())


############# OUTPUT PRINTER AND EMITTER #############


def print_output(line):
    """
    Handle a single line of output.
    """
    global vision_running
    gui_socket.emit("visionOutput", {"line": line})
    gui_socket.emit("visionStatus", {"status": vision_running})
    print(f"linha{line}", end="")


############# REFEREE BUTTON HANDLER #############


@gui_socket.on("refereeButton")
def handle_referee_button():
    pass


############# VISION BUTTON HANDLER #############


@gui_socket.on("visionButton")
def handle_vision_button():
    global vision_running, process
    command = ["ros2", "run", "vision", "visionNode"]
    if vision_running:
        vision_running = False
        gui_socket.emit("visionStatus", {"status": vision_running})
        gui_socket.emit("visionOutput", {"line": "Terminating vision node"})
        subprocess.run(["killall"] + [command[-1]])
    else:
        vision_running = True
        process = subprocess.Popen(
            command, stdout=subprocess.PIPE, stderr=subprocess.PIPE
        )

        # Create and start threads to handle stdout and stderr
        stdout_thread = Thread(
            target=handle_output, args=(process.stdout, print_output)
        )
        stderr_thread = Thread(
            target=handle_output, args=(process.stderr, print_output)
        )
        stdout_thread.start()
        stderr_thread.start()


############# COMMUNICATION BUTTON HANDLER #############


@gui_socket.on("communicationButton")
def handle_communication_button():
    pass


def main():
    global thread, vision_running
    thread = None
    vision_running = False
    gui_socket.run(app, debug=True, use_reloader=False, log_output=False)


if __name__ == "__main__":
    main()
