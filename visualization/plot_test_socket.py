import matplotlib.pyplot as plt
import time
import collections
import matplotlib.animation as animation
import threading
import re
import socket

# Replace with your ESP32's IP address
ESP32_IP = "192.168.180.63"  # Example IP address (replace with actual)
PORT = 8080

server_ip = ESP32_IP
port = PORT

# Create a socket object
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Connect to the ESP32 server
client_socket.connect((server_ip, port))
print(f"Connected to server at {server_ip}:{port}")

NUM_WHEELS = 4  # Number of wheels to plot

MAX_PWM = 255  # Maximum PWM value

# Set up the plot
fig, axs = plt.subplots(NUM_WHEELS, 2, figsize=(12, 10))  # Create subplots for 4 wheels (RPM and PWM)

# Initialize data structures for each wheel
wheels_data = {
    i: {
        'x_data': collections.deque(maxlen=100),
        'rpm_data': collections.deque(maxlen=100),
        'pwm_data': collections.deque(maxlen=100),
        'duty_cycle_data': collections.deque(maxlen=100),
        'target_rpm': 0
    } for i in range(0, NUM_WHEELS)
}

numerical = re.compile(r'[-+]?\d*\.\d+|\d+')
x = [0, 0, 0, 0]  # Time values
rpms = [0, 0, 0, 0]  # RPM values
pwms = [0, 0, 0, 0]  # PWM values
duty_cycles = [0, 0, 0, 0]  # Duty cycle values
target_rpms = [0, 0, 0, 0]  # Target RPM values

data_lock = threading.Lock()  # Lock to prevent race conditions

# Function to read data in the background
def read_serial_data():
    global x, rpms, pwms, target_rpms
    while True:
        # line = ser.readline().decode('utf-8', errors='ignore').strip()
        messages = client_socket.recv(1024).decode('utf-8')
        # extract to arrays of lines with '\n'
        lines_messages = messages.split('\n')

        for line in lines_messages:
            print(line)

            if line.startswith("Wheel_ID:"):
                # Extract data for each wheel
                parts = line.split()
                with data_lock:  # Acquire lock to safely update shared data
                    wheel_id = None
                    current_rpm = None
                    target_rpm = None
                    pwm_value = None
                    duty_cycle = None
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
                                duty_cycle = pwm_value / MAX_PWM * 100
                    print(wheel_id, current_rpm, target_rpm, pwm_value, duty_cycle)

                    if wheel_id >= 0 and wheel_id < NUM_WHEELS:
                        x[wheel_id] += 1  # Increment the x (time) value
                        rpms[wheel_id] = current_rpm
                        target_rpms[wheel_id] = target_rpm
                        pwms[wheel_id] = pwm_value
                        duty_cycles[wheel_id] = duty_cycle

        # time.sleep(0.5)  # Delay to simulate real-time data

# Initialize data structures for each wheel
lines = [[None, None] for _ in range(NUM_WHEELS)]  # To store line objects for RPM and PWM
hlines = [None] * NUM_WHEELS  # To store horizontal line objects for target RPM

def initialize_plot():
    global lines
    for i in range(NUM_WHEELS):
        # RPM plot
        lines[i][0], = axs[i, 0].plot([], [], label=f"RPM {i}", color=f"C{i}")
        hlines[i] = axs[i, 0].axhline(y=target_rpms[i], color='red', linestyle='--', label=f"Target RPM {i} = {target_rpms[i]}")
        axs[i, 0].set_xlabel("Time (s)")
        axs[i, 0].set_ylabel("RPM")
        axs[i, 0].set_title(f"Real-time Plot of RPM {i}")
        axs[i, 0].legend(loc="upper left")

        # Duty Cycle plot
        lines[i][1], = axs[i, 1].plot([], [], label=f"Duty Cycle {i}", color=f"C{i}")
        axs[i, 1].set_xlabel("Time (s)")
        axs[i, 1].set_ylabel("Dutyle Cycle (%)")
        axs[i, 1].set_title(f"Real-time Duty Cycle {i}")
        axs[i, 1].legend(loc="upper left")


def update_plot(frame):
    global x, rpms, pwms, wheels_data, duty_cycles, target_rpms
    # with data_lock:  # Acquire lock to safely access shared data
    for i in range(NUM_WHEELS):
        # Update data for plotting
        hlines[i].set_ydata(target_rpms[i])  # Update the horizontal line's position
        hlines[i].set_label(f"Target RPM {i} = {target_rpms[i]}") # Update label
        axs[i, 0].legend(loc="upper left") # Redraw the legend
        
        wheels_data[i]['x_data'].append(x[i])
        wheels_data[i]['rpm_data'].append(rpms[i])
        wheels_data[i]['duty_cycle_data'].append(duty_cycles[i])
        # Update RPM line
        lines[i][0].set_data(wheels_data[i]['x_data'], wheels_data[i]['rpm_data'])
        axs[i, 0].relim()  # Recompute limits
        axs[i, 0].autoscale_view()  # Autoscale view
        # axs[i, 0].set_ylim(-10, 400)  # Set a fixed y-axis range for RPM


        # Update Duty Cycle line
        lines[i][1].set_data(wheels_data[i]['x_data'], wheels_data[i]['duty_cycle_data'])
        axs[i, 1].relim()  # Recompute limits
        axs[i, 1].autoscale_view()  # Autoscale view
        axs[i, 1].set_ylim(-10, 110)

# Initialize plot elements
initialize_plot()

# Start the thread to read serial data
serial_thread = threading.Thread(target=read_serial_data, daemon=True)
serial_thread.start()

# Create the animation
ani = animation.FuncAnimation(fig, update_plot, interval=100, blit=False)

# Show the plot
plt.subplots_adjust(hspace=0.5)
plt.show()

# Close the serial connection when done
socket.close()
print("Closing connection...")
