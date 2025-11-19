#include <Arduino.h>
#include "hardware/compass.h"
#include "communication/i2c.h"

Compass::Compass(uint8_t address) : i2cAddress_(address) {}

Compass &Compass::begin()
{
  if (initialized_)
  {
    return *this;
  }

  I2CBus::instance().begin();

  return *this;
}

CompassReading Compass::read()
{
  Angle heading = readHeading();

  return CompassReading{
      heading,
      Direction::fromAngle(heading)};
}

void Compass::setNorthToHeading()
{
  Angle currentHeading = readHeading();
  headingOffset_ = currentHeading * -1;
}

Angle Compass::readHeading()
{
  byte high_byte = I2CBus::instance().readByte(i2cAddress_, 2);
  byte low_byte = I2CBus::instance().readByte(i2cAddress_, 3);

  int heading = (high_byte << 8) | low_byte; // Combine high and low bytes

  float headingDeg = heading / 10.0; // Convert to degrees (range is 0-3599 -> 0.0-359.9 degrees)

  return Angle(headingDeg);
}