/*
  This file is part of the Arduino_LSM6DSOX library.
  Copyright (c) 2021 Arduino SA. All rights reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "LSM6DSOX.h"

#define LSM6DSOX_ADDRESS            0x6A

#define LSM6DSOX_WHO_AM_I_REG       0X0F
#define LSM6DSOX_CTRL1_XL           0X10
#define LSM6DSOX_CTRL2_G            0X11

#define LSM6DSOX_STATUS_REG         0X1E

#define LSM6DSOX_CTRL4_C            0X13
#define LSM6DSOX_CTRL6_C            0X15
#define LSM6DSOX_CTRL7_G            0X16
#define LSM6DSOX_CTRL8_XL           0X17

#define LSM6DSOX_OUT_TEMP_L         0X20
#define LSM6DSOX_OUT_TEMP_H         0X21

#define LSM6DSOX_OUTX_L_G           0X22
#define LSM6DSOX_OUTX_H_G           0X23
#define LSM6DSOX_OUTY_L_G           0X24
#define LSM6DSOX_OUTY_H_G           0X25
#define LSM6DSOX_OUTZ_L_G           0X26
#define LSM6DSOX_OUTZ_H_G           0X27

#define LSM6DSOX_OUTX_L_XL          0X28
#define LSM6DSOX_OUTX_H_XL          0X29
#define LSM6DSOX_OUTY_L_XL          0X2A
#define LSM6DSOX_OUTY_H_XL          0X2B
#define LSM6DSOX_OUTZ_L_XL          0X2C
#define LSM6DSOX_OUTZ_H_XL          0X2D

#define LSM6DSOX_TIMESTAMP0         0X40
#define LSM6DSOX_INT1_CTRL          0X0D


LSM6DSOXClass::LSM6DSOXClass(TwoWire& wire, uint8_t slaveAddress) :
  _wire(&wire),
  _spi(NULL),
  _slaveAddress(slaveAddress)
{
}

LSM6DSOXClass::LSM6DSOXClass(SPIClass& spi, int csPin, int irqPin) :
  _wire(NULL),
  _spi(&spi),
  _csPin(csPin),
  _irqPin(irqPin),
  _spiSettings(10E6, MSBFIRST, SPI_MODE0)
{
}

LSM6DSOXClass::~LSM6DSOXClass()
{
}


void LSM6DSOXClass::init(){
    if (_spi != NULL) {
    pinMode(_csPin, OUTPUT);
    digitalWrite(_csPin, HIGH);
    _spi->begin();
  } else {
    _wire->begin();
  }

  if (!(readRegister(LSM6DSOX_WHO_AM_I_REG) == 0x6C || readRegister(LSM6DSOX_WHO_AM_I_REG) == 0x69)) {
    end();
  }

  resetDevice();

}

int LSM6DSOXClass::begin()
{
  
  // Enable interrupt on INT1 pin for accelerometer ready.
  writeRegister(LSM6DSOX_INT1_CTRL,0x81);
  writeRegister(0X0B, 0x80); // DEN should also be enabled.

  // Set accelerometer ODR  
  // writeRegister(LSM6DSOX_CTRL1_XL, 0x42); // 104 hz
  // writeRegister(LSM6DSOX_CTRL1_XL, 0x72); // 833 Hz
  // writeRegister(LSM6DSOX_CTRL1_XL, 0x92); // 3.33khz
  writeRegister(LSM6DSOX_CTRL1_XL, 0xA2); // 6.66 khz

  // Set gyroscope ODR 
  // writeRegister(LSM6DSOX_CTRL2_G, 0x90); // 3.33 kHz
  writeRegister(LSM6DSOX_CTRL2_G, 0xA0); // 6.66 kHz 

  // Enable gyroscope LPF1 stage.
  writeRegister(LSM6DSOX_CTRL4_C, 0x02); 

  // Gyroscope LPF1 BW selection.
  writeRegister(LSM6DSOX_CTRL6_C, 0x00); // 335.5 Hz
  // writeRegister(LSM6DSOX_CTRL6_C, 0x03); // 609 Hz

  // Enable gyroscope high pass filter and set cutoff to 16 mHz
  writeRegister(LSM6DSOX_CTRL7_G, 0x40);

  // Enable LPF2 for the accelerometer and select bandwith
  writeRegister(LSM6DSOX_CTRL8_XL, 0xE9); // BW: ODR/800


  return 1;
}

void LSM6DSOXClass::resetDevice(){ 
  writeRegister(0x12,0x01);
}

void LSM6DSOXClass::end()
{
  if (_spi != NULL) {
    _spi->end();
    digitalWrite(_csPin, LOW);
    pinMode(_csPin, INPUT);
  } else {
    writeRegister(LSM6DSOX_CTRL2_G, 0x00);
    writeRegister(LSM6DSOX_CTRL1_XL, 0x00);
    _wire->end();
  }
}

int LSM6DSOXClass::readAcceleration(float& x, float& y, float& z)
{
  int16_t data[3];

  if (!readRegisters(LSM6DSOX_OUTX_L_XL, (uint8_t*)data, sizeof(data))) {
    x = NAN;
    y = NAN;
    z = NAN;

    return 0;
  }

  x = data[0] * 4.0 / 32768.0;
  y = data[1] * 4.0 / 32768.0;
  z = data[2] * 4.0 / 32768.0;

  return 1;
}

int LSM6DSOXClass::readSensors(PacketBuffer& packet){
  
  int16_t data[6];

  if (!readRegisters(LSM6DSOX_OUTX_L_G, (uint8_t*)data, sizeof(data))) {
    packet.packet.values[0] = 0.0;
    packet.packet.values[1] = 0.0;
    packet.packet.values[2] = 0.0;
    packet.packet.values[3] = 0.0;
    packet.packet.values[4] = 0.0;
    packet.packet.values[5] = 0.0;

    return 0;
  }

  packet.packet.timestamp = micros();

  // Gx = data[0] * 250.0 / 32768.0 - gyroBias[0];
  // Gy = data[1] * 250.0 / 32768.0 - gyroBias[1];
  // Gz = data[2] * 250.0 / 32768.0 - gyroBias[2];

  // Ax = data[3] * 2.0 / 32768.0 - accBias[0];
  // Ay = data[4] * 2.0 / 32768.0 - accBias[1];
  // Az = data[5] * 2.0 / 32768.0 - accBias[2];

  // packet.packet.values[0] = data[0] * 8.75/1000  - gyroBias[0];
  // packet.packet.values[1] = data[1] * 8.75/1000  - gyroBias[1];
  // packet.packet.values[2] = data[2] * 8.75/1000  - gyroBias[2];
  // packet.packet.values[3] = data[3] * 0.061/1000 - accBias[0];
  // packet.packet.values[4] = data[4] * 0.061/1000 - accBias[1];
  // packet.packet.values[5] = data[5] * 0.061/1000 - accBias[2];
  
  packet.packet.values[0] = data[0] * 8.75/1000 ;
  packet.packet.values[1] = data[1] * 8.75/1000 ;
  packet.packet.values[2] = data[2] * 8.75/1000 ;
  packet.packet.values[3] = data[3] * 0.061/1000;
  packet.packet.values[4] = data[4] * 0.061/1000;
  packet.packet.values[5] = data[5] * 0.061/1000;


  return 1;
}

int LSM6DSOXClass::calibrateSensors(){
  const int calNum = 2500; 
  PacketBuffer sensorVals;
  float sensorSum[6] = {0,0,0,0,0,0};
  for( int count = 0; count < calNum; count += 1 ) { 
    readSensors(sensorVals);
    sensorSum[0] += sensorVals.packet.values[0];
    sensorSum[1] += sensorVals.packet.values[1];
    sensorSum[2] += sensorVals.packet.values[2];
    sensorSum[3] += sensorVals.packet.values[3];
    sensorSum[4] += sensorVals.packet.values[4];
    sensorSum[5] += sensorVals.packet.values[5];

    delayMicroseconds(250);
  }  


  gyroBias[0] = sensorSum[0]/calNum;
  gyroBias[1] = sensorSum[1]/calNum;
  gyroBias[2] = sensorSum[2]/calNum;
  accBias[0]  = sensorSum[3]/calNum;
  accBias[1]  = sensorSum[4]/calNum+1;
  accBias[2]  = sensorSum[5]/calNum;
  
  return 0;

}

int LSM6DSOXClass::accelerationAvailable()
{
  if (readRegister(LSM6DSOX_STATUS_REG) & 0x01) {
    return 1;
  }

  return 0;
}

float LSM6DSOXClass::accelerationSampleRate()
{
  return 104.0F;
}

int LSM6DSOXClass::readGyroscope(float& x, float& y, float& z)
{
  int16_t data[3];

  if (!readRegisters(LSM6DSOX_OUTX_L_G, (uint8_t*)data, sizeof(data))) {
    x = NAN;
    y = NAN;
    z = NAN;

    return 0;
  }

  x = data[0] * 2000.0 / 32768.0;
  y = data[1] * 2000.0 / 32768.0;
  z = data[2] * 2000.0 / 32768.0;

  return 1;
}

int LSM6DSOXClass::gyroscopeAvailable()
{
  if (readRegister(LSM6DSOX_STATUS_REG) & 0x02) {
    return 1;
  }

  return 0;
}

int LSM6DSOXClass::readTemperature(int& temperature_deg)
{
  float temperature_float = 0;
  readTemperatureFloat(temperature_float);

  temperature_deg = static_cast<int>(temperature_float);

  return 1;
}

int LSM6DSOXClass::readTemperatureFloat(float& temperature_deg)
{
  /* Read the raw temperature from the sensor. */
  int16_t temperature_raw = 0;

  if (readRegisters(LSM6DSOX_OUT_TEMP_L, reinterpret_cast<uint8_t*>(&temperature_raw), sizeof(temperature_raw)) != 1) {
    return 0;
  }

  /* Convert to °C. */
  static int const TEMPERATURE_LSB_per_DEG = 256;
  static int const TEMPERATURE_OFFSET_DEG = 25;

  temperature_deg = (static_cast<float>(temperature_raw) / TEMPERATURE_LSB_per_DEG) + TEMPERATURE_OFFSET_DEG;

  return 1;
}

int LSM6DSOXClass::temperatureAvailable()
{
  if (readRegister(LSM6DSOX_STATUS_REG) & 0x04) {
    return 1;
  }

  return 0;
}

float LSM6DSOXClass::gyroscopeSampleRate()
{
  return 104.0F;
}

int LSM6DSOXClass::readRegister(uint8_t address)
{
  uint8_t value;
  
  if (readRegisters(address, &value, sizeof(value)) != 1) {
    return -1;
  }
  
  return value;
}

int LSM6DSOXClass::readRegisters(uint8_t address, uint8_t* data, size_t length)
{
  if (_spi != NULL) {
    _spi->beginTransaction(_spiSettings);
    digitalWrite(_csPin, LOW);
    _spi->transfer(0x80 | address);
    _spi->transfer(data, length);
    digitalWrite(_csPin, HIGH);
    _spi->endTransaction();
  } else {
    _wire->beginTransmission(_slaveAddress);
    _wire->write(address);

    if (_wire->endTransmission(false) != 0) {
      return -1;
    }

    if (_wire->requestFrom(_slaveAddress, length) != length) {
      return 0;
    }

    for (size_t i = 0; i < length; i++) {
      *data++ = _wire->read();
    }
  }
  return 1;
}

int LSM6DSOXClass::writeRegister(uint8_t address, uint8_t value)
{
  if (_spi != NULL) {
    _spi->beginTransaction(_spiSettings);
    digitalWrite(_csPin, LOW);
    _spi->transfer(address);
    _spi->transfer(value);
    digitalWrite(_csPin, HIGH);
    _spi->endTransaction();
  } else {
    _wire->beginTransmission(_slaveAddress);
    _wire->write(address);
    _wire->write(value);
    if (_wire->endTransmission() != 0) {
      return 0;
    }
  }
  return 1;
}


int LSM6DSOXClass::readSensorsWTS(PacketBuffer& packet){


  int16_t data[6];
  uint32_t timestamp;

  readRegisters(LSM6DSOX_OUTX_L_G, (uint8_t*)data, sizeof(data));
  readRegisters(LSM6DSOX_TIMESTAMP0, (uint8_t*)(&timestamp), sizeof(timestamp));

  packet.packet.timestamp = timestamp*25;
  
  packet.packet.values[0] = data[0] * 8.75/1000 ;
  packet.packet.values[1] = data[1] * 8.75/1000 ;
  packet.packet.values[2] = data[2] * 8.75/1000 ;
  packet.packet.values[3] = data[3] * 0.061/1000;
  packet.packet.values[4] = data[4] * 0.061/1000;
  packet.packet.values[5] = data[5] * 0.061/1000;


  return 1;


}









#ifdef LSM6DS_DEFAULT_SPI
LSM6DSOXClass IMU_LSM6DSOX(LSM6DS_DEFAULT_SPI, PIN_SPI_SS1, LSM6DS_INT);
#else
LSM6DSOXClass IMU_LSM6DSOX(Wire, LSM6DSOX_ADDRESS);
#endif