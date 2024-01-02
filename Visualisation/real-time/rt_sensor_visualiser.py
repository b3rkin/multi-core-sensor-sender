import select
import socket
import struct
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import time


# Arduino IP address and port
#arduino_ip = "192.168.4.1"  # Replace with the Arduino's IP address
arduino_ip = "192.168.1.123"
arduino_port = 2390  # Replace with the Arduino's UDP port

# UDP server configuration
UDP_IP = "0.0.0.0"  # Replace with the IP address you want to listen on
UDP_PORT = 5500       # Replace with the port you want to listen on

# Initialize UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind((UDP_IP, UDP_PORT))


# Send a UDP packet to the Arduino to start transmission.
message = "Start"
sock.sendto(message.encode(), (arduino_ip, arduino_port))

# Sensor data configuration
SENSOR_COUNT = 6     # Number of sensors (gyroX, gyroY, gyroZ, accX, accY, accZ)
SENSOR_SIZE = 4      # Size of each sensor value in bytes
TIMESTAMP_SIZE = 4   # Size of the timestamp in bytes
PACKET_SIZE = SENSOR_COUNT * SENSOR_SIZE + TIMESTAMP_SIZE  # Total packet size in bytes

# Plot configuration
WINDOW_SIZE = 300    # Number of data points to show in the sliding window
PLOT_INTERVAL = 0.001  # Time delay between plot updates in seconds


# Initialize plot
# fig, axs = plt.subplots(SENSOR_COUNT, 1, sharex=True)
fig, axes = plt.subplots(SENSOR_COUNT, 1, figsize=(10, 10))

for i in range(SENSOR_COUNT):
    axes[i].clear()
    axes[i].set_xlabel('Timestamp')
    axes[i].set_ylabel('Sensor {}'.format(i+1))
    axes[i].set_title('Sensor {}'.format(i+1))

axes[0].set_ylim(-250, 250)
axes[0].set_ylabel('gyroX (deg/s)')

axes[1].set_ylim(-250, 250)
axes[1].set_ylabel('gyroY (deg/s)')

axes[2].set_ylim(-250, 250)
axes[2].set_ylabel('gyroY (deg/s)')

axes[3].set_ylim(-2, 2)
axes[3].set_ylabel('accX (g)')

axes[4].set_ylim(-2, 2)
axes[4].set_ylabel('accY (g)')

axes[5].set_ylim(-2, 2)
axes[5].set_ylabel('accZ (g)')

plt.tight_layout()
# Initialize data for each sensor
sensor_data = [deque(maxlen=WINDOW_SIZE) for _ in range(SENSOR_COUNT)]
timestamps = [deque(maxlen=WINDOW_SIZE)]

# Initialize line objects for each subplot
lines = [ax.plot([], [])[0] for ax in axes]

def empty_socket(sock):
    """remove the data present on the socket"""
    input = [sock]
    while 1:
        inputready, o, e = select.select(input,[],[], 0.0)
        if len(inputready)==0: break
        for s in inputready: s.recv(1)


# Function to update the plot
def update_plot(frame, sensor_data, timestamps):

    # Receive UDP packet
    data, addr = sock.recvfrom(PACKET_SIZE)

    # Decode packet data
    for i in range(SENSOR_COUNT):
        sensor_value = struct.unpack_from('f', data, i * SENSOR_SIZE)[0]
        sensor_data[i].append(sensor_value)

    timestamp = struct.unpack_from('I', data, SENSOR_COUNT * SENSOR_SIZE)[0]
    timestamps[0].append(timestamp)


    # Update each line object
    for i, line in enumerate(lines):
        line.set_data(timestamps[0], sensor_data[i])
        if not len(sensor_data[i]) < 2:
    #         axes[i].set_ylim(min(sensor_data[i])*1.5, max(sensor_data[i])*1.5)
            axes[i].set_xlim(timestamps[0][0], timestamps[0][-1])
    # fig.canvas.draw()

    # Empty the socket
    empty_socket(sock)

    return lines


# Create the animation
ani = animation.FuncAnimation(fig, update_plot, fargs = (sensor_data, timestamps), interval= 1,  blit=True)

# Show the plot
plt.show()



