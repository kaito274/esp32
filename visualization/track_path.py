import matplotlib.pyplot as plt
import matplotlib.animation as animation
import random
import math
import time
import serial
import re
import threading

# Open the serial port (make sure the port is correct)
ser = serial.Serial('COM3', 115200)  # Replace 'COM4' with your port
time.sleep(2)  # Wait for ESP32 to reset

NUM_WHEELS = 4  # Number of wheels to plot

numerical = re.compile(r'[-+]?\d*\.\d+|\d+')

# Initialize data for car position tracking
x_positions = []
y_positions = []
directions = []  # Direction in degrees (0 degrees = right, 90 degrees = up)

x = 0
y = 0
direction = 0


data_lock = threading.Lock()  # Lock to prevent race conditions

# Function to read data in the background
def read_serial_data():
    global x, y, direction
    while True:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        print(line)
        if line.startswith("Point_x:"):
            # Extract data for each wheel
            parts = line.split()
            with data_lock:  # Acquire lock to safely update shared data
                for part in parts:
                    if "Point_x" in part:
                        match = re.search(numerical, part)
                        if match:
                            x = float(match.group(0))
                    elif "Point_y" in part:
                        match = re.search(numerical, part)
                        if match:
                            y = float(match.group(0))
                    elif "Direction_Angle" in part:
                        match = re.search(numerical, part)
                        if match:
                            direction = float(match.group(0))

        # sleep(2)

# Function to update the plot with random data
def update_position(frame):
    global x, y, direction, x_positions, y_positions, directions
    # Add a new random data point to simulate car position
    new_x = x 
    new_y = y 
    new_direction = direction
    # new_direction = 180

    x_positions.append(new_x)
    y_positions.append(new_y)
    directions.append(new_direction % 360)  # Keep direction in [0, 360) degrees

    # time.sleep(0.25)

    # Clear the current plot
    plt.cla()

    # Scatter plot the car's trajectory
    plt.scatter(x_positions, y_positions, color='blue', label='Car Path', s=10)  # Small dots for trajectory
    plt.scatter([x_positions[-1]], [y_positions[-1]], color='red', label='Current Position', s=50)  # Highlight current positiongit 

    # Add an arrow indicating the car's current direction
    arrow_length = 0.5  # Consistent length of the arrow
    arrow_dx = arrow_length * math.cos(math.radians(directions[-1]))
    arrow_dy = arrow_length * math.sin(math.radians(directions[-1]))
    plt.arrow(x_positions[-1], y_positions[-1], arrow_dx, arrow_dy, head_width=0.2, head_length=0.2, fc='green', ec='green', label='Direction')

    plt.xlabel("X Coordinate")
    plt.ylabel("Y Coordinate")
    plt.title("Car Trajectory with Direction")
    plt.legend(loc="upper left")
    plt.grid(True)

# # Create the figure
# fig = plt.figure(figsize=(6, 6))

# # Create the animation with a faster interval (100ms)
# ani = animation.FuncAnimation(fig, update_position, interval=100)

# # Show the plot
# plt.show()


# Start the thread to read serial data
serial_thread = threading.Thread(target=read_serial_data, daemon=True)
serial_thread.start()

# Create the animation
ani = animation.FuncAnimation(plt.gcf(), update_position, interval=100, blit=False)

# Show the plot
plt.show()

# Close the serial connection when done
ser.close()
print("Serial connection closed.")

