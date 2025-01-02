import matplotlib.pyplot as plt
import matplotlib.animation as animation
import random
import math
import time

# Initialize data for car position tracking
x_positions = []
y_positions = []
directions = []  # Direction in degrees (0 degrees = right, 90 degrees = up)

# Function to update the plot with random data
def update_position(frame):
    # Add a new random data point to simulate car position
    new_x = x_positions[-1] + random.uniform(-1, 1) if x_positions else 0
    new_y = y_positions[-1] + random.uniform(-1, 1) if y_positions else 0
    new_direction = directions[-1] + random.uniform(-30, 30) if directions else 0  # Random direction change
    # new_direction = 180

    x_positions.append(new_x)
    y_positions.append(new_y)
    directions.append(new_direction % 360)  # Keep direction in [0, 360) degrees

    time.sleep(0.25)

    # Clear the current plot
    plt.cla()

    # Scatter plot the car's trajectory
    plt.scatter(x_positions, y_positions, color='blue', label='Car Path', s=10)  # Small dots for trajectory
    plt.scatter([x_positions[-1]], [y_positions[-1]], color='red', label='Current Position', s=50)  # Highlight current position

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

# Create the figure
fig = plt.figure(figsize=(6, 6))

# Create the animation with a faster interval (100ms)
ani = animation.FuncAnimation(fig, update_position, interval=100)

# Show the plot
plt.show()
