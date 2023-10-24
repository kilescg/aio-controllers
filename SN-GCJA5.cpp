#include "SN-GCJA5.h"

//Test to see if the device is connected
bool SFE_PARTICLE_SENSOR::begin(TwoWire &wirePort)
{
  _i2cPort = &wirePort; //Grab which port the user wants us to use

  return (isConnected()); //Check to see if device acks to its address.
}

//Given a mass density PM register, do conversion and return mass density
std::pair<int, float> SFE_PARTICLE_SENSOR::getPM(uint8_t pmRegister)
{
  auto count = readRegister32(pmRegister);
  return std::make_pair(count.first, (count.second / 1000.0f));
}
std::pair<int, float> SFE_PARTICLE_SENSOR::getPM1_0()
{
  return (getPM(SNGCJA5_PM1_0));
}
std::pair<int, float> SFE_PARTICLE_SENSOR::getPM2_5()
{
  return (getPM(SNGCJA5_PM2_5));
}
std::pair<int, float> SFE_PARTICLE_SENSOR::getPM10()
{
  return (getPM(SNGCJA5_PM10));
}

//Particle count functions
std::pair<int, uint16_t> SFE_PARTICLE_SENSOR::getPC0_5()
{
  return (readRegister16(SNGCJA5_PCOUNT_0_5));
}
std::pair<int, uint16_t> SFE_PARTICLE_SENSOR::getPC1_0()
{
  return (readRegister16(SNGCJA5_PCOUNT_1_0));
}
std::pair<int, uint16_t> SFE_PARTICLE_SENSOR::getPC2_5()
{
  return (readRegister16(SNGCJA5_PCOUNT_2_5));
}
std::pair<int, uint16_t> SFE_PARTICLE_SENSOR::getPC5_0()
{
  return (readRegister16(SNGCJA5_PCOUNT_5_0));
}
std::pair<int, uint16_t> SFE_PARTICLE_SENSOR::getPC7_5()
{
  return (readRegister16(SNGCJA5_PCOUNT_7_5));
}
std::pair<int, uint16_t> SFE_PARTICLE_SENSOR::getPC10()
{
  return (readRegister16(SNGCJA5_PCOUNT_10));
}

//State functions
std::pair<int, uint8_t> SFE_PARTICLE_SENSOR::getState()
{
  return (readRegister8(SNGCJA5_STATE));
}
std::pair<int, uint8_t> SFE_PARTICLE_SENSOR::getStatusSensors()
{
  auto state = getState();
  return std::make_pair(state.first, ((state.second >> 6) & 0b11));
}
std::pair<int, uint8_t> SFE_PARTICLE_SENSOR::getStatusPD()
{
  auto state = getState();
  return std::make_pair(state.first, ((state.second >> 4) & 0b11));
}
std::pair<int, uint8_t> SFE_PARTICLE_SENSOR::getStatusLD()
{
  auto state = getState();
  return std::make_pair(state.first, ((state.second >> 2) & 0b11));
}
std::pair<int, uint8_t> SFE_PARTICLE_SENSOR::getStatusFan()
{
  auto state = getState();
  return std::make_pair(state.first, ((state.second >> 0) & 0b11));
}

//Low level I2C functions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//Returns true if device acknowledges its address
bool SFE_PARTICLE_SENSOR::isConnected()
{
  _i2cPort->beginTransmission(_deviceAddress);
  if (_i2cPort->endTransmission() != 0)
    return (false); //Sensor did not ACK
  return (true);    //All good
}

//Reads from a given location
//Stores the result at the provided outputPointer
std::pair<int, uint8_t> SFE_PARTICLE_SENSOR::readRegister8(uint8_t addr)
{
  _i2cPort->beginTransmission(_deviceAddress);
  _i2cPort->write(addr);
  if (_i2cPort->endTransmission(false) != 0)
    return std::make_pair(SNGCJA5_I2C_NO_ACK, 0); //Sensor did not ACK

  _i2cPort->requestFrom((uint8_t)_deviceAddress, (uint8_t)1);
  if (_i2cPort->available())
    return std::make_pair(SNGCJA5_ERROR_NONE, (_i2cPort->read()));

  return std::make_pair(SNGCJA5_I2C_NO_RESP, 0); //Sensor did not respond
}

//Reads two consecutive bytes from a given location
std::pair<int, uint16_t> SFE_PARTICLE_SENSOR::readRegister16(uint8_t addr)
{
  _i2cPort->beginTransmission(_deviceAddress);
  _i2cPort->write(addr);
  if (_i2cPort->endTransmission(false) != 0)
    return std::make_pair(SNGCJA5_I2C_NO_ACK, 0); //Sensor did not ACK

  _i2cPort->requestFrom((uint8_t)_deviceAddress, (uint8_t)2);
  if (_i2cPort->available())
  {
    uint8_t lsb = _i2cPort->read();
    uint8_t msb = _i2cPort->read();
    return std::make_pair(SNGCJA5_ERROR_NONE, ((uint16_t)msb << 8 | lsb));
  }
  return std::make_pair(SNGCJA5_I2C_NO_RESP, 0); //Sensor did not respond
}

//Reads four consecutive bytes from a given location
std::pair<int, uint32_t> SFE_PARTICLE_SENSOR::readRegister32(uint8_t addr)
{
  _i2cPort->beginTransmission(_deviceAddress);
  _i2cPort->write(addr);
  if (_i2cPort->endTransmission(false) != 0)
    return std::make_pair(SNGCJA5_I2C_NO_ACK, 0); //Sensor did not ACK

  _i2cPort->requestFrom((uint8_t)_deviceAddress, (uint8_t)4);
  if (_i2cPort->available())
  {
    uint8_t ll = _i2cPort->read();
    uint8_t lh = _i2cPort->read();
    uint8_t hl = _i2cPort->read();
    uint8_t hh = _i2cPort->read();
    return std::make_pair(SNGCJA5_ERROR_NONE, (((uint32_t)hh << 24) | ((uint32_t)hl << 16) | ((uint32_t)lh << 8) | ((uint32_t)ll << 0)));
  }
  return std::make_pair(SNGCJA5_I2C_NO_RESP, 0); //Sensor did not respond
}
