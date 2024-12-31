import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time
import collections
import threading
import re

# Open the serial port (make sure the port is correct)
ser = serial.Serial('COM4', 115200)  # Replace 'COM4' with your port
time.sleep(2)  # Wait for ESP32 to reset

NUM_WHEELS = 4  # Number of wheels to plot

# Set up the plot
fig, axs = plt.subplots(NUM_WHEELS, 1, figsize=(8, 10))  # Create subplots for 4 wheels

# Initialize data structures for each wheel
wheels_data = {
    i: {
        'x_data': collections.deque(maxlen=100),
        'y_data': collections.deque(maxlen=100),
        'target_rpm': 0
    } for i in range(0, NUM_WHEELS)
}

numerical = re.compile(r'[-+]?\d*\.\d+|\d+')
x = [0, 0, 0, 0]  # Time values
rpms = [0, 0, 0, 0]  # RPM values
target_rpms = [0, 0, 0, 0]  # Target RPM values

data_lock = threading.Lock()  # Lock to prevent race conditions

# Function to read data in the background
def read_serial_data():
    global x, rpms, target_rpms
    while True:
        line = ser.readline().decode('utf-8', errors='ignore').strip()

        if line.startswith("Wheel_ID:"):
            # Extract data for each wheel
            parts = line.split()
            with data_lock:  # Acquire lock to safely update shared data
                wheel_id = None
                current_rpm = None
                target_rpm = None
                for part in parts:
                    if "Wheel_ID" in part:
                        match = re.search(numerical, part)
                        if match:
                            wheel_id = int(match.group(0))
                    elif "Current_RPM" in part:
                        match = re.search(numerical, part)
                        if match:
                            current_rpm = float(match.group(0))
                    elif "Target_RPM" in part:
                        match = re.search(numerical, part)
                        if match:
                            target_rpm = float(match.group(0))

                if wheel_id >= 0 and wheel_id < NUM_WHEELS:
                    x[wheel_id] += 1  # Increment the x (time) value
                    rpms[wheel_id] = current_rpm
                    target_rpms[wheel_id] = target_rpm


# Function to update the plot with new data
def update_plot(frame):
    global x, rpms, target_rpms, wheels_data
    with data_lock:  # Acquire lock to safely access shared data
        for i in range(0, NUM_WHEELS):
            # Append the new data for plotting
            wheels_data[i]['x_data'].append(x[i])
            wheels_data[i]['y_data'].append(rpms[i])

    for i in range(0, NUM_WHEELS):
        # Clear the axis before updating the plot
        axs[i].cla()

        # Plot the updated data
        axs[i].plot(wheels_data[i]['x_data'], wheels_data[i]['y_data'], label=f"RPM {i}", color=f"C{i}")

        # Plot the horizontal line for target RPM
        axs[i].axhline(y=wheels_data[i]['target_rpm'], color='red', linestyle='--', label=f"Target RPM {i} ({wheels_data[i]['target_rpm']})")

        # Set labels and title
        axs[i].set_xlabel("Time (s)")
        axs[i].set_ylabel("RPM")
        axs[i].set_title(f"Real-time Plot of RPM {i}")

        # Redraw the plot
        axs[i].legend(loc="upper left")

        # Fit the plot to the data
        axs[i].relim()

# Start the thread to read serial data
serial_thread = threading.Thread(target=read_serial_data, daemon=True)
serial_thread.start()

# Create the animation
ani = animation.FuncAnimation(fig, update_plot, interval=100, blit=False)

# Show the plot
plt.subplots_adjust(hspace=0.5)
# plt.tight_layout()
plt.show()

# Close the serial connection when done
ser.close()
print("Serial connection closed.")
