#pragma once
#include <Arduino.h>
#include <algorithm>
#include "util/directional.h"
#include "pin.h"
#include "communication/i2c.h"

struct CompassReading
{
public:
  Angle heading;
  Direction headingDirection;
};

class Compass
{
public:
  enum class Reg : uint8_t
  {
    HeadingHigh = 0x02,
    HeadingLow = 0x03,
  };

  Compass(uint8_t i2cAddress);

  Compass &begin();

  CompassReading read();
  void setNorthToHeading();

private:
  bool initialized_ = false;

  uint8_t i2cAddress_ = 0x60;

  Angle headingOffset_ = Angle(0);

  Angle readHeading();
};
