import serial
import matplotlib.pyplot as plt
import time
import collections
import matplotlib.animation as animation
import threading
import re

# Open the serial port (make sure the port is correct)
ser = serial.Serial('COM4', 115200)  # Replace 'COM5' with your port
time.sleep(2)  # Wait for ESP32 to reset

# Check if the serial connection is open
if ser.is_open:
    print("Serial connection open.")


# Set up the plot
plt.figure(figsize=(10, 6))  # Set the figure size (width=10, height=6)
x_data = collections.deque(maxlen=100)  # Use a deque to hold the data with a maximum length
y_data = collections.deque(maxlen=100)

numerical = re.compile(r'[-+]?\d*\.\d+|\d+')
x = 0
position = 0
target_position = 0
data_lock = threading.Lock()  # Lock to prevent race conditions

# Function to read data in the background
def read_serial_data():
    global position, target_position, x
    while True:
        line = ser.readline().decode('utf-8', errors='ignore').strip()

        if line.startswith("Target_position:"):
            # Extract x and y values
            parts = line.split()
            with data_lock:  # Acquire lock to update shared data safely
                for part in parts:
                    if "Current_position" in part:
                        match = re.search(r"[-+]?\d*\.\d+|\d+", part)
                        if match:
                            position = float(match.group(0))
                    elif "Target_position" in part:
                        match = re.search(r"[-+]?\d*\.\d+|\d+", part)
                        if match:
                            target_position = float(match.group(0))
                x += 1  # Increment the x (time) value

# Function to update the plot with new data
def update_plot(frame):
    global x, position, target_position
    
    with data_lock:  # Acquire lock to safely access shared data
        # Append the new data for plotting
        x_data.append(x)
        y_data.append(position)

    # Clear the axis before updating the plot
    plt.clf()

    # Plot the updated data
    plt.plot(x_data, y_data, label="Position", color="blue")
    
    # Plot the horizontal line for target Position
    plt.axhline(y=target_position, color='red', linestyle='--', label=f"Target Position ({target_position})")
    
    # Set labels and title
    plt.xlabel("Time (ms)")
    plt.ylabel("Position")
    plt.title("Real-time Plot of Position")

    # # Update axis limits only when data is available
    # if len(x_data) > 1:  # Ensure we have more than one data point
    #     plt.xlim([min(x_data) - 1, max(x_data) + 1])  # Set x-limits to ensure smooth scrolling
    #     plt.ylim([min(y_data) - 1, max(y_data) + 1])  # Set y-limits dynamically based on data

    # Redraw the plot with updated data
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
