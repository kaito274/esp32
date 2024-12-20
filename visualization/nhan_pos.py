import serial
import matplotlib.pyplot as plt
import time
import collections
import matplotlib.animation as animation
import threading
import re
import socket

# Replace with your ESP32's IP address
ESP32_IP = "192.168.1.10"  # Example IP address (replace with actual)
PORT = 8080

server_ip = ESP32_IP
port = PORT

# Open the serial port (make sure the port is correct)
# ser = serial.Serial('COM5', 115200)  # Replace 'COM5' with your port
# time.sleep(2)  # Wait for ESP32 to reset
# Create a socket object
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Connect to the ESP32 server
client_socket.connect((server_ip, port))
print(f"Connected to server at {server_ip}:{port}")
while True:
    # Receive data from the ESP32
    # read string data from the ESP32
    line = client_socket.recv(1024).decode('utf-8')


# Close the serial connection when done
socket.close()
print("Closing connection...")
