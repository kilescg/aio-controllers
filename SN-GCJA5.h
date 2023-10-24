#ifndef SN-GCJA5_H
#define SN-GCJA5_H

#include "Arduino.h"

#include <Wire.h>

class SFE_PARTICLE_SENSOR
{
  public:

    enum SNGCJA5_ERRORS {
      SNGCJA5_ERROR_NONE  = 0,
      SNGCJA5_I2C_NO_ACK  = -1,
      SNGCJA5_I2C_NO_RESP = -2,
    };

    enum SNGCJA5_REGISTERS {
      SNGCJA5_PM1_0 = 0x00,
      SNGCJA5_PM2_5 = 0x04,
      SNGCJA5_PM10 = 0x08,
      SNGCJA5_PCOUNT_0_5 = 0x0C,
      SNGCJA5_PCOUNT_1_0 = 0x0E,
      SNGCJA5_PCOUNT_2_5 = 0x10,
      SNGCJA5_PCOUNT_5_0 = 0x14,
      SNGCJA5_PCOUNT_7_5 = 0x16,
      SNGCJA5_PCOUNT_10 = 0x18,
      SNGCJA5_STATE = 0x26,
    };

    //By default use the default I2C address, and use Wire port
    bool begin(TwoWire &wirePort = Wire);
    bool isConnected();

    std::pair<int, uint8_t> readRegister8(uint8_t addr);
    std::pair<int, uint16_t> readRegister16(uint8_t addr);
    std::pair<int, uint32_t> readRegister32(uint8_t addr);

    std::pair<int, float> getPM(uint8_t pmRegister);
    std::pair<int, float> getPM1_0();
    std::pair<int, float> getPM2_5();
    std::pair<int, float> getPM10();

    std::pair<int, uint16_t> getPC0_5();
    std::pair<int, uint16_t> getPC1_0();
    std::pair<int, uint16_t> getPC2_5();
    std::pair<int, uint16_t> getPC5_0();
    std::pair<int, uint16_t> getPC7_5();
    std::pair<int, uint16_t> getPC10();

    std::pair<int, uint8_t> getState();
    std::pair<int, uint8_t> getStatusSensors();
    std::pair<int, uint8_t> getStatusPD();
    std::pair<int, uint8_t> getStatusLD();
    std::pair<int, uint8_t> getStatusFan();

  private:
    TwoWire *_i2cPort; //The generic connection to user's chosen I2C hardware
    uint8_t _deviceAddress = 0x33; //Default, unchangable address
};

#endif // SN-GCJA5_H
