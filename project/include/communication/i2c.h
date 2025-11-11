#pragma once
#include <Arduino.h>

class I2CBus
{
public:
  I2CBus &begin();

  byte readByte(uint8_t address, uint8_t reg);

private:
  bool initialized_ = false;
};