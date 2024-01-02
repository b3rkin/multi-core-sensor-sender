
#define NETWORK_HOST true

#include <WiFiNINA.h>
#include "src/libraries/Arduino_LSM6DSOX/src/Arduino_LSM6DSOX.h"
#include "lib/wifi.hpp"
#include "lib/led.hpp"
#include "lib/structs.hpp"
#include "arduino_secrets.h"
#include <atomic>
#include <SPI.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"

#define INT1_PIN 24
#define AVERAGING_SAMPLE_NUM 6

LedController LED;
WifiController * wifi;
PacketBuffer packetBufferSensor;
PacketBuffer packetBufferAverage;
PacketBuffer packetBufferWifi;

volatile bool readFlag = false;
volatile bool sendFlag = false;
int dataCounter = 0;
int counter = 0;

void setup(){

    Serial.begin(115200); // Start Serial connection.
    LED.init(); // Initialise the LEDs
    LED.set(ledColor::BLUE);
    #if NETWORK_HOST
    wifi = new WifiController(); // Initialize 
    wifi->waitUntilConnectionToAP();
    #else
    wifi = new WifiController(SECRET_SSID, SECRET_PASS); // Initialize 
    wifi->waitUntilConnectedToAP();
    #endif
    wifi->waitForFirstReceivedPacket();
    LED.clear(ledColor::BLUE);
    LED.set(ledColor::GREEN);
    
    rp2040.fifo.push_nb(1); // Send start signal to the other core.
    
    
}

void setup1(){

    delay(100);
    Wire.setClock(3000000);
    IMU.init();
    delay(100);
    pinMode(INT1_PIN, INPUT);
    rp2040.fifo.pop(); // Wait for the start signal from the other core. (waits until available)
    noInterrupts(); // Disable interrupts to avoid race conditions
    attachInterrupt(digitalPinToInterrupt(INT1_PIN), readSensorsOnInterrupt, RISING);
    IMU.begin();
    interrupts(); // Re-enable interrupts

}

void loop() {


    while(true){
        if(sendFlag){
            sendFlag = false;
            if(counter < 2000){    // Discard first few IMU samples (I suspect there is a startup period)
                packetBufferWifi = receiveFromOtherCore();
                counter++;
            }else{
                break;
            }
        }
    }

    while(true){ // Continuously send sensor data.
        if(sendFlag){
            
            if(rp2040.fifo.available() == 8){
                packetBufferWifi = receiveFromOtherCore();
                wifi->sendSensorData(packetBufferWifi);
                sendFlag = false;
            }
        }
    }
}


void loop1(){
    
    noInterrupts(); // Disable interrupts to avoid race conditions
    if(readFlag){

        readFlag = false;

          IMU.readSensors(packetBufferSensor);
          dataCounter++;
          
          if(dataCounter == 1){
                  packetBufferAverage.packet = packetBufferSensor.packet;
          }else{
                  packetBufferAverage.packet += packetBufferSensor.packet;
          }

          if(dataCounter == AVERAGING_SAMPLE_NUM){
              for(int i = 0; i<6; i++){
                  packetBufferAverage.packet.values[i] = packetBufferAverage.packet.values[i]/AVERAGING_SAMPLE_NUM;
              }
              packetBufferAverage.packet.timestamp = packetBufferAverage.packet.timestamp/AVERAGING_SAMPLE_NUM;
              
              sendToOtherCore(packetBufferAverage);
              sendFlag = true;
              dataCounter = 0;
          }
    }
    interrupts(); // Re-enable interrupts
}


void readSensorsOnInterrupt() {
    readFlag = true;
}


void sendToOtherCore(const PacketBuffer& packet) {
    uint32_t* data = (uint32_t*)packet.buffer;
    size_t size = sizeof(SensorOutput) / sizeof(uint32_t);


    for (size_t i = 0; i < size; i++) {
        // Push each uint32_t to the FIFO
        rp2040.fifo.push_nb(data[i]);
    }
}

PacketBuffer receiveFromOtherCore() {
    PacketBuffer packet;
    
    uint32_t* data = (uint32_t*)packet.buffer;
    size_t size = sizeof(SensorOutput) / sizeof(uint32_t);
    for (size_t i = 0; i < size; i++) {
        // Pop each uint32_t from the FIFO and store it
        data[i] = rp2040.fifo.pop();
    }

    return packet;
}


