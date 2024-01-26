import socket
import eventlet
import socketio
import threading
import eventlet.wsgi
from flask import Flask

sio = socketio.Server()
app = Flask(__name__)

MAX_SPEED = 35
MIN_SPEED = 20

speed_limit = MAX_SPEED
steering_angle = 0


@sio.on('telemetry')
def telemetry(sid, data):
    if data:
        global speed_limit

        speed = float(data['speed'])
        if speed > speed_limit:
            speed_limit = MIN_SPEED
        else:
            speed_limit = MAX_SPEED
        
        throttle = 1.0 - steering_angle ** 2 - (speed / speed_limit) ** 2
        send_control(steering_angle, throttle)
    else:
        # NOTE: DON'T EDIT THIS.
        sio.emit('manual', data={}, skip_sid=True)


@sio.on('connect')
def connect(sid, environ):
    print(f"Simulator: {sid} just connected to the HEAD POSE BASED GAME CONTROLLER 🦲🚘")
    print()
    send_control(0, 1)


def send_control(steering_angle, throttle):
    sio.emit(
        'steer',
        data={
            'steering_angle': str(steering_angle),
            'throttle': str(throttle)
        },
        skip_sid=True
    )

def start_simulator_server(host='', port=4567):
    global app

    app = socketio.Middleware(sio, app)
    eventlet.wsgi.server(eventlet.listen((host, port)), app)

def start_simulator_control_server(host='localhost', port=5555):
    global steering_angle

    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((host, port))
    server_socket.listen(1)

    while True:
        print("Waiting for a steering signal from the client...")
        client_socket, address = server_socket.accept()
        steering_signal = client_socket.recv(1024)
        if steering_signal:
            steering_signal = steering_signal.decode()
        steering_angle = float(steering_signal)
        print(f"Client: {address} just sent a steering signal: {steering_angle} to the simulator")
        print()
        client_socket.close()
        

if __name__ == '__main__':
    simulator_server_thread = threading.Thread(target=start_simulator_server)
    simulator_control_thread = threading.Thread(target=start_simulator_control_server)

    simulator_server_thread.start()
    simulator_control_thread.start()

    simulator_server_thread.join()
    simulator_control_thread.join()
    