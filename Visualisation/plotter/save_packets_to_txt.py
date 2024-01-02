import socket

# Arduino IP address and port
arduino_ip = "192.168.4.1"  # Replace with the Arduino's IP address
arduino_port = 2390  # Replace with the Arduino's UDP port

# Create a UDP socket

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

# Bind the socket to a specific IP address and port (optional)
# This is only required if you want to receive UDP packets on your computer
local_ip = "0.0.0.0"  # Use 0.0.0.0 to bind to all available network interfaces
local_port = 5500  # Replace with the port you want to receive UDP packets on
sock.bind((local_ip, local_port))

# Send a UDP packet to the Arduino
message = "Start"
sock.sendto(message.encode(), (arduino_ip, arduino_port))

# Clear File
with open('packets.txt', 'w') as file:
    print("file cleaned")

# Receive UDP packets from the Arduino
while True:
    data, addr = sock.recvfrom(28)  # Buffer size is 1024 bytes
    # print("Received message from Arduino:", data)
    with open('packets.txt', 'a') as file:
        file.write(repr(data))
        file.write('\n')

# Close the socket (optional)
sock.close()