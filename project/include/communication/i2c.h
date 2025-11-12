#pragma once
#include <Arduino.h>

class I2CBus
{
public:
  // Singleton access
  static I2CBus &instance()
  {
    static I2CBus instance;
    return instance;
  }

  I2CBus &begin();

  byte readByte(uint8_t address, uint8_t reg);

private:
  I2CBus() {} // Hide constructor for singleton pattern

  bool initialized_ = false;
};