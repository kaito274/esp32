from math import cos, pi, radians, sin
import socket
import json
import time
import numpy as np
# import numpy as np


# Replace with your ESP32's IP address and port
ESP32_IP = "192.168.215.63"
PORT = 8080

server_ip = ESP32_IP
port = PORT

# Create a socket object
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
print(f"Connecting to server at {server_ip}:{port}...")

# Connect to the ESP32 server
client_socket.connect((server_ip, port))
print(f"Connected to server at {server_ip}:{port}")

time_history = [0]
x_history = [0]
y_history = [0]
phi_history = [0]

v1_history = [0]
v2_history = [0]
v3_history = [0]
v4_history = [0]
vCarX_history = [0]
vCarY_history = [0]
vCarPhi_history = [0]

phi1_history = [0]
phi2_history = [0]
phi3_history = [0]
phi4_history = [0]

l = 0.25; # Distance between wheel pairs (m)
d = 0.25; # Distance between wheels along the axis (m)
wheelR = 0.05; 

# Inverse kinematics matrix 
J_inv = [[1, -1, -(l + d)],
         [1,  1, (l + d)],
         [1, 1, -(l + d)],
         [1, -1, (l + d)]]

def start_move(distance, angleMove, angleSpin, time, step):
  global time_history

  dt = time / step
  time_array = [i * dt for i in range(step)]
  # print(time_array)
  time_history_last = time_history[-1]
  time_history += ([round(i + time_history_last, 2) for i in time_array[1:]])
  
  angleMove = radians(angleMove)
  angleSpin = radians(angleSpin)
  vCarX = distance * cos(angleMove) / time
  vCarY = distance * sin(angleMove) / time
  vCarPhi = angleSpin / time
  
  # wheelAngularVelocities = [vCarX, vCarY, vCarPhi] * J_inv / wheelR
  v1, v2, v3, v4 = (1 / wheelR) * (vCarX - vCarY - (l + d) * vCarPhi), \
                    (1 / wheelR) * (vCarX + vCarY + (l + d) * vCarPhi), \
                    (1 / wheelR) * (vCarX + vCarY - (l + d) * vCarPhi), \
                    (1 / wheelR) * (vCarX - vCarY + (l + d) * vCarPhi)

  print(f'vcarX: {vCarX}, vcarY: {vCarY}, vcarPhi: {vCarPhi}') 
  for i in range(step):
    dPhi = vCarPhi * dt
    phi = phi_history[-1] + dPhi
    x_pos, y_pos = x_history[-1] + (vCarX * cos(phi) - vCarY * sin(phi)) * dt, \
                  y_history[-1] + (vCarX * sin(phi) + vCarY * cos(phi)) * dt
    
    phi1_pos, phi2_pos, phi3_pos, phi4_pos = phi1_history[-1] + v1 * dt, \
                                              phi2_history[-1] + v2 * dt, \
                                              phi3_history[-1] + v3 * dt, \
                                              phi4_history[-1] + v4 * dt
    
    v1_history.append(v1)
    v2_history.append(v2)
    v3_history.append(v3)
    v4_history.append(v4)
    phi1_history.append(phi1_pos)
    phi2_history.append(phi2_pos)
    phi3_history.append(phi3_pos)
    phi4_history.append(phi4_pos)
    
    vCarX_history.append(vCarX)
    vCarY_history.append(vCarY)
    vCarPhi_history.append(vCarPhi)
    x_history.append(x_pos)
    y_history.append(y_pos)
    phi_history.append(phi)
    
def square_with_turn(distance, timePerOp, stepPerOp):
  # start_move(distance, 0, 0, timePerOp, stepPerOp)
  # start_move(0, 0, 90, timePerOp, stepPerOp)
  for i in range(4):
    start_move(distance, 0, 0, timePerOp, stepPerOp)
    start_move(0, 0, 90/2, timePerOp, stepPerOp)
    

def square_no_turn(distance, timePerOp, stepPerOp):
  for i in range(4):
    start_move(distance, 90 * (i), 0, timePerOp, stepPerOp) 


def circle_with_turn(radius, timePerOp, stepPerOp):
  start_move(2 * pi * radius, 0, 360 / 2, timePerOp, stepPerOp)


def circle_center_turn(radius, timePerOp, stepPerOp):
  # % step 1: rotate left / right
  # move(0, 0, -90, timePerOp, 10)

  # % step 2: move horizontally + rotate bit by bit
  # move(radius * 2 * pi , 90, 360, timePerOp, stepPerOp); % 5m circle 180 degree turn
  
  start_move(0, 0, -90 / 2, timePerOp, stepPerOp)
  start_move(2 * pi * radius, 90, 360 / 2, timePerOp, stepPerOp)


def circle_no_turn(radius, timePerOp, stepPerOp):
  totalDeg = 360
  unitDeg = totalDeg / stepPerOp
  unitArc = radius * radians(unitDeg)
  
  # for angle in range(1, totalDeg, unitDeg):
  for angle in np.arange(0, totalDeg, unitDeg):
    start_move(unitArc, angle, 0, timePerOp / stepPerOp, 1)


def start_roll(opt, distance, time_duration, step):
  global phi1_history, phi2_history, phi3_history, phi4_history

  if opt == 0:
    square_with_turn(distance, time_duration, step)
  elif opt == 1:
    square_no_turn(distance, time_duration, step)
  elif opt == 2:
    circle_with_turn(distance, time_duration, step)
  elif opt == 3:
    circle_center_turn(distance, time_duration, step)
  elif opt == 4:
    circle_no_turn(distance, time_duration, step)
  
  print('> before')
  print(phi1_history, '\n', phi2_history, '\n', phi3_history, '\n', phi4_history)
  
  # convert to pulse
  phi1_history = [round(i * 330 / (2 * pi)) for i in phi1_history]
  phi2_history = [round(i * 330 / (2 * pi) * -1) for i in phi2_history]
  phi3_history = [round(i * 330 / (2 * pi)) for i in phi3_history]
  phi4_history = [round(i * 330 / (2 * pi) * -1) for i in phi4_history]
  
  # calculate delta
  phi1_history = [phi1_history[i] - phi1_history[i - 1] for i in range(1, len(phi1_history))]
  phi2_history = [phi2_history[i] - phi2_history[i - 1] for i in range(1, len(phi2_history))]
  phi3_history = [phi3_history[i] - phi3_history[i - 1] for i in range(1, len(phi3_history))]
  phi4_history = [phi4_history[i] - phi4_history[i - 1] for i in range(1, len(phi4_history))]

  # print('> velocities')
  # print(v1_history, '\n', v2_history, '\n', v3_history, '\n', v4_history)

  print('> after')
  print(phi1_history, '\n', phi2_history, '\n', phi3_history, '\n', phi4_history)

  # send data to ESP32 each 1 seconds
  for i in range(len(phi1_history)):
    data_to_send = {
      'p1': phi1_history[i],
      'p2': phi2_history[i],
      'p3': phi3_history[i],
      'p4': phi4_history[i]
    }
    # Convert dictionary to JSON string
    json_data = json.dumps(data_to_send)

    try:
      # Send JSON string
      client_socket.send((json_data + "\n").encode('utf-8'))  # Append '\n' as a delimiter
      print(f"Sent: {json_data}")
    except BrokenPipeError:
      print("Connection lost. Could not send data.")
      break

    time.sleep(1)  # Send data every 1 second
    

# for square
start_roll(1, 0.5, 1, 4)

# for circle
# start_roll(4, 0.5, 1, 32)

# function square_with_turn(distance, timePerOp, stepPerOp) 
#     % global timePerOp stepPerOp;

#     for i = 1:4
#         % Move straight
#         move(distance, 0, 0, timePerOp, stepPerOp);

#         % Turn 90°
#         move(0, 0, 90, timePerOp, stepPerOp);
#     end
  
# print(time_history)
# print(vCarX_history, '\n', vCarY_history, '\n', vCarPhi_history)
# print(x_history, '\n', y_history, '\n', phi_history)

# phi1_history = phi1_history / (2 * pi)
# phi2_history = phi2_history / (2 * pi)
# phi3_history = phi3_history / (2 * pi)
# phi4_history = phi4_history / (2 * pi)

# phi1_history = [i / (2 * pi) for i in phi1_history]
# phi2_history = [i / (2 * pi) for i in phi2_history]
# phi3_history = [i / (2 * pi) for i in phi3_history]
# phi4_history = [i / (2 * pi) for i in phi4_history]

  