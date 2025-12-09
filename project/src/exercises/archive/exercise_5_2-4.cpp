#include <LiquidCrystal.h>
#include <Arduino.h>
#include "exercises.h"
#include "hardware.h"
#include "physical.h"
#include <Wire.h>

namespace ex_5_2
{

#define CMPS14_address 0x60 // I2C slave address for COMPS module, default:0xC0 >> 0x60

  namespace p = physical;

  void setup()
  {
    // put your setup code here, to run once:
    Serial.begin(9600);

    p::lcd::device.begin();

    Wire.begin();
  }

  byte readByte(byte address, byte reg)
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

  float offsetDegrees(float degrees, float offsetDegrees)
  {
    float adjustedDegrees = degrees + offsetDegrees;
    if (adjustedDegrees >= 360.0)
    {
      adjustedDegrees -= 360.0;
    }
    else if (adjustedDegrees < 0.0)
    {
      adjustedDegrees += 360.0;
    }
    return adjustedDegrees;
  }

  float readHeadingDegrees()
  {
    byte high_byte = readByte(CMPS14_address, 2);
    byte low_byte = readByte(CMPS14_address, 3);

    int heading = (high_byte << 8) | low_byte; // Combine high and low bytes

    return heading / 10.0; // Convert to degrees (range is 0-3599 -> 0.0-359.9 degrees)
  }

  String getHeadingDirection(float degrees)
  {
    degrees = fmod(degrees, 360.0);
    if (degrees < 0)
      degrees += 360.0;

    if (degrees >= 23 && degrees < 68)
      return "NE";
    else if (degrees >= 68 && degrees < 113)
      return "E";
    else if (degrees >= 113 && degrees < 158)
      return "SE";
    else if (degrees >= 158 && degrees < 203)
      return "S";
    else if (degrees >= 203 && degrees < 248)
      return "SW";
    else if (degrees >= 248 && degrees < 293)
      return "W";
    else if (degrees >= 293 && degrees < 338)
      return "NW";
    else // covers 338–360 and 0–23
      return "N";
  }

  void loop()
  {

    float heading = offsetDegrees(readHeadingDegrees(), 180);
    String direction = getHeadingDirection(heading);
    p::lcd::device.printLine(2, "Heading " + direction + " :" + String(heading, 2));

    delay(200);
  }
}