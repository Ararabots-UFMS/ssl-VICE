from flask import Flask
from flask_socketio import SocketIO, emit
from threading import Thread
import subprocess
import select
import math
import os
import signal

app = Flask(__name__)
socketio = SocketIO(app,  cors_allowed_origins="*")
global vision_running

@socketio.on("connect")
def handle_connect():
    print("Client connected")
    global thread
    global stop_thread
    stop_thread = False
    if thread is None:
        thread = Thread(target=test_thread)
        thread.start()

@socketio.on("disconnect")
def handle_disconnect():
    print("Client disconnected")
    global thread
    global stop_thread
    stop_thread = True
    thread = None

@socketio.on("fieldSide")
def handle_field_side(side):
    #TODO get side to rest of the code
    if side == True:
        print("Field is on the left side", flush=True)
    elif side == False:
        print("Field is on the right side", flush=True)

@socketio.on("teamColor")
def handle_field_side(color):
    #TODO get color to rest of the code
    if color == True:
        print("Team color is blue", flush=True)
    elif color == False:
        print("Team color is yellow", flush=True)

@socketio.on("refereeButton")
def handle_referee_button():
    pass

def handle_output(pipe, callback):
    """
    Read output from a pipe and pass it to a callback line by line.
    """
    with pipe:
        for line in iter(pipe.readline, b''):
            callback(line.decode())

def print_output(line):
    """
    Handle a single line of output.
    """
    socketio.emit("visionOutput", {"line": line})
    print(f'linha{line}', end='')

@socketio.on("visionButton")
def handle_vision_button():
    global vision_running, process
    command = ['ros2', 'run', 'demo_nodes_py', 'talker']
    if vision_running:
        vision_running = False
        socketio.emit("visionOutput", {"line": "Terminating vision node"})
        subprocess.run(['killall']+[command[-1]])

    else:
        vision_running = True
        command = ['ros2', 'run', 'demo_nodes_py', 'talker']
        process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

        # Create and start threads to handle stdout and stderr
        stdout_thread = Thread(target=handle_output, args=(process.stdout, print_output))
        stderr_thread = Thread(target=handle_output, args=(process.stderr, print_output))
        stdout_thread.start()
        stderr_thread.start()



@socketio.on("communicationButton")
def handle_communication_button():
    pass

@socketio.on("message")
def handle_message(message):
    print(f"Received message: {message}", flush=True)

def test_thread():

    x = 0
    y = 0
    radius = 50
    angle = 0

    while not stop_thread:
        x = 250+radius * math.cos(angle)
        y = 250+radius * math.sin(angle)
        angle += 0.1
        socketio.emit("position", {"x": x, "y": y, "angle": angle})
        # print(f"Sent update: x={x}, y={y}", flush=True)
        socketio.sleep(0.1)  # Adjust the delay as needed

def main():
    global thread, vision_running
    thread = None
    vision_running = False
    socketio.run(app, debug=True)

if __name__ == "__main__":
    main()