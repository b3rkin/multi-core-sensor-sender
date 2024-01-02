#ifndef STRUCTS_HPP
#define STRUCTS_HPP


struct SensorOutput
{
  float values[6];   // sensorArray sensor;
  uint32_t timestamp;   // timestamp packetTime;
  uint8_t padding[4];  

    // Overload the =+ operator
  SensorOutput& operator+=(const SensorOutput& other) {

    for (int i = 0; i < 6; i++) {
        this->values[i] = this->values[i] + other.values[i];
    }

    this->timestamp = this->timestamp + other.timestamp; // Always pick the RHS timestamp (Assumed to be the larger one)
    return *this;
  }
    
};

union PacketBuffer
{
  SensorOutput packet;
  uint8_t buffer[sizeof(SensorOutput)];
};


#endif