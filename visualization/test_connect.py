import socket
import matplotlib.pyplot as plt

def connect_to_esp32(server_ip, port):
    try:
        # Create a socket object
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        # Connect to the ESP32 server
        client_socket.connect((server_ip, port))
        print(f"Connected to server at {server_ip}:{port}")

        # # Real-time data storage
        # all_data = [[], [], [], []]
        # max_points = 50  # Maximum number of points to display on the plot at a time

        # # Set up real-time plot
        # plt.ion()  # Enable interactive mode
        # fig, axs = plt.subplots(4, 1, figsize=(10, 8))
        # lines = []

        # for i in range(4):
        #     line, = axs[i].plot([], [], label=f"Data {i + 1}", color=f"C{i}")
        #     axs[i].set_xlim(0, max_points)
        #     axs[i].set_ylim(0, 100)  # Adjust based on expected data range
        #     axs[i].set_title(f"Data {i + 1} Over Time")
        #     axs[i].set_xlabel("Time (samples)")
        #     axs[i].set_ylabel("Values")
        #     axs[i].legend()
        #     axs[i].grid()
        #     lines.append(line)

        # while True:
        #     # Receive data from the ESP32
        #     data = client_socket.recv(1024).decode('utf-8')

        #     if not data:
        #         break

        #     # Check if the stop signal is received
        #     if data.strip() == "20":
        #         print("Stop signal received. Closing connection.")
        #         break

        #     # Split the received data into individual values
        #     values = data.split(',')
        #     print(f"Received data: {values}")

        #     if len(values) == 4:  # Ensure the data contains exactly 4 values
        #         for i in range(4):
        #             all_data[i].append(int(values[i]))

        #             # Keep only the last `max_points` data points
        #             if len(all_data[i]) > max_points:
        #                 all_data[i].pop(0)

        #         # Update the plots
        #         for i in range(4):
        #             lines[i].set_xdata(range(len(all_data[i])))
        #             lines[i].set_ydata(all_data[i])
        #             axs[i].relim()
        #             axs[i].autoscale_view()

        #         # Pause to update the figure
        #         plt.pause(0.1)

        # client_socket.close()
        # print("Connection closed.")

        # # Keep the final plot open
        # plt.ioff()
        # plt.show()

    except Exception as e:
        print(f"An error occurred: {e}")

if __name__ == "__main__":
    # Replace with your ESP32's IP address
    ESP32_IP = "192.168.1.7"  # Example IP address (replace with actual)
    PORT = 8080

    connect_to_esp32(ESP32_IP, PORT)