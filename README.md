# The Project:
This project is written in the Arduino language for the Arduino Nano RP2040 Connect. The purpose of this project is to create a wifi access point on the Arduino wifi chip and send accelerometer and gyroscope values using UDP packets to the device that connects to this access point.

## Current status of the project:
The Arduino creates a WIFI access point called "tracked-object-AP" and then starts waiting for a connection. Once a device is connected to the AP, the Arduino expects an initial UDP packet (Any random packet will do) to starts its operation. There are 3 available options/functions described below. 

## Setup
This program specifically requires the Arduino Nano RP2040 board. To run this program the code can directly be compiled and uploaded to the board via the Arduino IDE, given that the required Arduino libraries stated in the dependencies section are installed. 
1. Download the complete code to desired location on your computer.
2. Open the "object-sensor-sender.ino" file with the Arduino IDE (v1 or v2).
3. Connect the Arduino Nano RP2040 Connect to your machine via USB.
4. Be sure to have installed required Arduino libraries (WiFiNINA).
5. Select the correct board and port that is connected in the Arduino IDE. 
    - The RP2040 Connect should show up automatically once connected.
6. Press the upload sketch button, the program will compile, upload and start automatically.
7. If the Arduino will be run on battery power without a connected USB cable the "USB_DEBUG_MODE" should be turned of before uploading in "object_sensor_sender.ino". The USB cable should never be connected to a computer when the Arduino runs on batteries.

## Functions
| Function              | Purpose       |
|---------------------|-------------------------------|
|  sendSensorData() |    The device will sample the accelerometer and gyroscope and send the data in a UDP packet to the selected port continuously.              |
|    speedTestUDP()     |       The device will sample the sensors and send 1000 packets over UDP and print how long it took.          |
|  timeTestUDP()  |     The device will sample the sensors and send packets during 1 second over UDP and print how many packets have been sent.       |



## Packet Format
The packet that is sent over UDP is formatted as follows:

| gyroscope X | gyroscope Y |gyroscope Z| accelerometer X | accelerometer Y | accelerometer Z| timestamp|
|-------------|-------------|-----------|-----------------|-----------------|----------------|----------|
| 4 bytes float | 4 bytes float | 4 bytes float| 4 bytes float | 4 bytes float | 4 bytes float|4 bytes unsigned long|

## Command List
By connecting to the arduino WiFi access point from another device or the main device, commands can be sent over UDP at all times. The following commands are available. 
| Command #             | Command       |
|---------------------|-------------------------------|
|    1     |       Start continuous mode operation.   |
|  2 |     Stop operation and go to "await command" mode .   |
## Modes
## Units


## The coordinate system

## Data Visualisation
Currently there are 2 python scripts available for data visualisation. One writes the raw incoming UDP packets to a file per line, the second visualises the incoming sensor data in real time. In both cases the PC needs to be connected to the Arduino via the "tracked-object-AP" access point before calling the scripts.
1. Connect PC to Arduino Hotspot "tracked-object-AP" with password: "password".
2. Run the script.
### save_packets_to_txt.py
This script continuously saves the received UDP packets per line in a file called "packets.txt"
### rt_sensor_visualiser.py
This script visualises the 6 incoming sensor values from the Arduino in real-time.
## TO-DO
 - [ ] Add more operating modes and configurations.
   - [ ] Add LED color scheme based on operating mode.
 - [ ] Add several commands such as start and stop that can be send over UDP to the arduino to be able to control the operation over WiFi.  
    - [ ] Add a "start" and "stop" command.
 - [ ] Add Setup info to README.
  - [ ] Add visualisation guide.
 ## Dependencies
 1. The Arduino code needs an installed version of the following Arduino Libraries:
    - WiFiNINA
 2. The visualisation scripts require:
    - matplotlib
    - numpy
 ## References
 1. Sensor code fork of: https://github.com/arduino-libraries/Arduino_LSM6DSOX