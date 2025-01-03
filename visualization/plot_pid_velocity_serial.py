import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time
import collections
import threading
import re

# Open the serial port (make sure the port is correct)
ser = serial.Serial('COM4', 115200)  # Replace 'COM5' with your port
time.sleep(2)  # Wait for ESP32 to reset

# Set up the plot
plt.figure(figsize=(10, 6))  # Set the figure size (width=10, height=6)
x_data = collections.deque(maxlen=100)  # Use a deque to hold the data with a maximum length
y_data = collections.deque(maxlen=100)

numerical = re.compile(r'[-+]?\d*\.\d+|\d+')
x = 0
rpm = 0
target_rpm = 0
data_lock = threading.Lock()  # Lock to prevent race conditions

# Lock to prevent race conditions
data_lock = threading.Lock()

# Function to read data in the background
def read_serial_data():
    global rpm, target_rpm, x
    while True:
        line = ser.readline().decode('utf-8', errors='ignore').strip()

        if line.startswith("Wheel_ID:0"):
            # Extract x and y values
            parts = line.split()
            # print(parts)
            with data_lock:  # Acquire lock to safely update shared data
                for part in parts:
                    if "Current_RPM" in part:
                        match = re.search(r"[-+]?\d*\.\d+|\d+", part)
                        if match:
                            rpm = float(match.group(0))  # Convert matched value to float
                    elif "Target_RPM" in part:
                        match = re.search(r"[-+]?\d*\.\d+|\d+", part)
                        if match:
                            target_rpm = float(match.group(0))  # Convert matched value to float
                x += 1  # Increment the x (time) value

# Function to update the plot with new data
def update_plot(frame):
    global x, rpm, target_rpm
    with data_lock:  # Acquire lock to safely access shared data
        # Append the new data for plotting
        x_data.append(x)
        y_data.append(rpm)

    # Clear the axis before updating the plot
    plt.clf()

    # Plot the updated data
    plt.plot(x_data, y_data, label="RPM", color="blue")
    
    # Plot the horizontal line for target RPM
    plt.axhline(y=target_rpm, color='red', linestyle='--', label=f"Target RPM ({target_rpm})")
    
    # Set labels and title
    plt.xlabel("Time (s)")
    plt.ylabel("RPM")
    plt.title("Real-time Plot of RPM")

    # # Update axis limits only when data is available
    # if len(x_data) > 1:  # Ensure we have more than one data point
    #     plt.xlim([min(x_data) - 1, max(x_data) + 1])  # Set x-limits to ensure smooth scrolling
    #     plt.ylim([min(y_data) - 1, max(y_data) + 1])  # Set y-limits dynamically based on data

    # Redraw the plot
    plt.legend(loc="upper left")

# Start the thread to read serial data
serial_thread = threading.Thread(target=read_serial_data, daemon=True)
serial_thread.start()

# Create the animation
ani = animation.FuncAnimation(plt.gcf(), update_plot, interval=100, blit=False)

# Show the plot
plt.show()

# Close the serial connection when done
ser.close()
print("Serial connection closed.")
