import socket
import threading
import time

# Replace with your ESP32's IP address and port
ESP32_IP = "192.168.180.63"
PORT = 8080

server_ip = ESP32_IP
port = PORT

# Create a socket object
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Connect to the ESP32 server
client_socket.connect((server_ip, port))
print(f"Connected to server at {server_ip}:{port}")

# Function to send data to the ESP32
def send_data():
    while True:
        # Create the data to send (customize this based on your needs)
        data_to_send = "Message from Python: Hello, ESP32!"
        try:
            # Send data
            client_socket.send(data_to_send.encode('utf-8'))
            print(f"Sent: {data_to_send}")
        except BrokenPipeError:
            print("Connection lost. Could not send data.")
            break

        time.sleep(1)  # Send data every 1 second

# Function to receive data from the ESP32
def receive_data():
    while True:
        try:
            line = client_socket.recv(1024).decode('utf-8')
            print(f"Received: {line}")
        except Exception as e:
            print(f"Error receiving data: {e}")
            break

# Start threads for sending and receiving data
send_thread = threading.Thread(target=send_data, daemon=True)
receive_thread = threading.Thread(target=receive_data, daemon=True)

send_thread.start()
receive_thread.start()

# Keep the main thread alive
try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("Closing connection...")
    client_socket.close()
