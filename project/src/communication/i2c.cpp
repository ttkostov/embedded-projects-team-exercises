#include <Arduino.h>
#include "communication/i2c.h"
#include <Wire.h>

I2CBus &I2CBus::begin()
{
  if (initialized_)
    return *this;

  initialized_ = true;

  Wire.begin();

  return *this;
}

byte I2CBus::readByte(uint8_t address, uint8_t reg)
{
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.endTransmission(false);

  Wire.requestFrom(address, 1, (int)true);

  if (Wire.available())
  {
    return Wire.read();
  }
  return 0;
}