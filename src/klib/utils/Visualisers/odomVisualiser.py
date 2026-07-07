import serial
import matplotlib.pyplot as plt
import math

# Adjust port based on connection
PORT = 'COM8'
BAUD = 115200

ser = serial.Serial(PORT, BAUD)

x_data = []
y_data = []

plt.ion()
fig, ax = plt.subplots()

# Initialise an Empty Line for Path
line, = ax.plot([], [])

while True:
    try:
        line_raw = ser.readline().decode().strip()
        x, y, theta = map(float, line_raw.split(','))

        x_data.append(x)
        y_data.append(y)

        # Update Path
        line.set_xdata(x_data)
        line.set_ydata(y_data)

        # Clear Arrows
        ax.collections.clear()

        # Draw Heading Error
        ax.quiver(
            x, y,
            math.cos(theta),
            math.sin(theta),
            angles='xy',
            scale_units='xy',
            scale=1
        )

        # Field
        ax.set_xlim(-72, 72)
        ax.set_ylim(-72, 72)

        ax.set_title("Odometry Visualisation")
        ax.set_xlabel("X (inches)")
        ax.set_ylabel("Y (inches)")

        plt.pause(0.001)

    except Exception as e:
        print("Parse error:", e)