from flask import Flask
from flask_socketio import SocketIO, emit
from threading import Thread
import math

app = Flask(__name__)
socketio = SocketIO(app,  cors_allowed_origins="*")

@socketio.on("connect")
def handle_connect():
    print("Client connected")
    global thread
    global stop_thread
    stop_thread = False
    if thread is None:
        thread = Thread(target=background_thread)
        thread.start()

@socketio.on("disconnect")
def handle_disconnect():
    global thread
    global stop_thread
    stop_thread = True
    thread = None

@socketio.on("message")
def handle_message(message):
    print(f"Received message: {message}", flush=True)

def background_thread():

    x = 0
    y = 0
    radius = 50
    angle = 0

    while not stop_thread:
        x = 250+radius * math.cos(angle)
        y = 250+radius * math.sin(angle)
        angle += 0.1
        socketio.emit("position", {"x": x, "y": y})
        # print(f"Sent update: x={x}, y={y}", flush=True)
        socketio.sleep(0.1)  # Adjust the delay as needed

def main():
    global thread
    thread = None
    socketio.run(app, debug=True)

if __name__ == "__main__":
    main()