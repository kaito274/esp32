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
fig, axs = plt.subplots(NUM_WHEELS, 2, figsize=(12, 10))  # Create subplots for 4 wheels (RPM and PWM)

# Initialize data structures for each wheel
wheels_data = {
    i: {
        'x_data': collections.deque(maxlen=100),
        'rpm_data': collections.deque(maxlen=100),
        'pwm_data': collections.deque(maxlen=100),
        'target_rpm': 0
    } for i in range(0, NUM_WHEELS)
}

numerical = re.compile(r'[-+]?\d*\.\d+|\d+')
x = [0, 0, 0, 0]  # Time values
rpms = [0, 0, 0, 0]  # RPM values
pwms = [0, 0, 0, 0]  # PWM values
target_rpms = [0, 0, 0, 0]  # Target RPM values

data_lock = threading.Lock()  # Lock to prevent race conditions

# Function to read data in the background
def read_serial_data():
    global x, rpms, pwms, target_rpms
    while True:
        line = ser.readline().decode('utf-8', errors='ignore').strip()

        if line.startswith("Wheel_ID:"):
            # Extract data for each wheel
            parts = line.split()
            with data_lock:  # Acquire lock to safely update shared data
                wheel_id = None
                current_rpm = None
                target_rpm = None
                pwm_value = None
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
                    elif "PWM" in part:
                        match = re.search(numerical, part)
                        if match:
                            pwm_value = float(match.group(0))

                if wheel_id >= 0 and wheel_id < NUM_WHEELS:
                    x[wheel_id] += 1  # Increment the x (time) value
                    rpms[wheel_id] = current_rpm
                    target_rpms[wheel_id] = target_rpm
                    pwms[wheel_id] = pwm_value

# Function to update the plot with new data
def update_plot(frame):
    global x, rpms, pwms, target_rpms, wheels_data
    with data_lock:  # Acquire lock to safely access shared data
        for i in range(0, NUM_WHEELS):
            # Append the new data for plotting
            wheels_data[i]['x_data'].append(x[i])
            wheels_data[i]['rpm_data'].append(rpms[i])
            wheels_data[i]['pwm_data'].append(pwms[i])

    for i in range(0, NUM_WHEELS):
        # Clear the RPM axis before updating the plot
        axs[i, 0].cla()
        axs[i, 1].cla()

        # Plot the RPM data
        axs[i, 0].plot(wheels_data[i]['x_data'], wheels_data[i]['rpm_data'], label=f"RPM {i}", color=f"C{i}")
        axs[i, 0].axhline(y=wheels_data[i]['target_rpm'], color='red', linestyle='--', label=f"Target RPM {i} ({wheels_data[i]['target_rpm']})")
        axs[i, 0].set_xlabel("Time (s)")
        axs[i, 0].set_ylabel("RPM")
        axs[i, 0].set_title(f"Real-time Plot of RPM {i}")
        axs[i, 0].legend(loc="upper left")

        # Plot the PWM data
        axs[i, 1].plot(wheels_data[i]['x_data'], wheels_data[i]['pwm_data'], label=f"PWM {i}", color=f"C{i}")
        axs[i, 1].set_xlabel("Time (s)")
        axs[i, 1].set_ylabel("PWM")
        axs[i, 1].set_title(f"Real-time Plot of PWM {i}")
        axs[i, 1].legend(loc="upper left")

# Start the thread to read serial data
serial_thread = threading.Thread(target=read_serial_data, daemon=True)
serial_thread.start()

# Create the animation
# ani = animation.FuncAnimation(fig, update_plot, interval=100, blit=False)
ani = animation.FuncAnimation(fig, update_plot, interval=100, blit=False)

# Show the plot
plt.subplots_adjust(hspace=0.5)
plt.show()

# Close the serial connection when done
ser.close()
print("Serial connection closed.")
