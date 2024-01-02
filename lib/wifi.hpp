#ifndef WIFI_HPP
#define WIFI_HPP

#include <WiFiNINA.h>
#include "structs.hpp"
#include "../arduino_secrets.h"

class WifiController{

    private:

        //******************** Wifi Parameters ********************//

        // The IP address of this device generally becomes 196.168.4.1
        unsigned int webServerPort; // (80) I don't think this is needed for UDP
        unsigned int localPort;     // (2390) This is the port the main devices should listen to.
        unsigned int remotePort;    // (5500) This is the port where this arduino sends UDP Packets to.

        WiFiServer server;
        IPAddress remoteIP;
        WiFiUDP Udp;

        int status = WL_IDLE_STATUS;

        char ssid[18] = {"tracked-object-AP"};        // Network SSID (name)
        char pass[9] = {"password"};                 // Network password 

        //*********************************************************//
        // Memeber for sending IMU data.
        PacketBuffer sensorData;

    public:

    WifiController(): 
        webServerPort(80),
        localPort(2390),
        remotePort(5500),
        server(webServerPort)
        {
            status = WiFi.beginAP(ssid, pass);
            // Wait 5 seconds for connection:
            delay(1000);
            // Start the web server on port 80.
            server.begin();
            Udp.begin(localPort); // Start a UDP server on port "localPort".
        }

    WifiController(auto ssid_arg, auto pass_arg): 
        webServerPort(80),
        localPort(2390),
        remotePort(5500),
        server(webServerPort)
        {
            status = WiFi.begin(ssid_arg, pass_arg);
            // Wait 1 second for connection:
            delay(1000);
            // Start the web server on port 80.
            server.begin();
            Udp.begin(localPort); // Start a UDP server on port "localPort".
        }

    void waitUntilConnectionToAP(){
        // Wait for the other device to connect to the access point.
        while(status != WL_AP_CONNECTED){
            delay(200);
            status = WiFi.status();
        }
    }

    void waitUntilConnectedToAP(){
        // Wait for the device to connect to the access point.
        while(status != WL_CONNECTED){
            delay(200);
            status = WiFi.status();
        }
    }

    void waitForFirstReceivedPacket(){
        // Wait for the first UDP Packet to arrive for transmission to start.
        while(1){
            int packetSize = Udp.parsePacket();
            if (packetSize) {
                IPAddress remoteIp = Udp.remoteIP();
                break;
            }
        }
        // Get the remote IP and port that sended the UDP Starting message.
        remoteIP = Udp.remoteIP();
        remotePort = Udp.remotePort();
    }

    void sendPacketUDP(uint8_t *buffer, size_t length){
        Udp.beginPacket(remoteIP, remotePort);
        Udp.write(buffer, length);
        Udp.endPacket();
    }

    void sendSensorData(PacketBuffer sensorData){
        sendPacketUDP(sensorData.buffer, 28);
    }


};

#endif