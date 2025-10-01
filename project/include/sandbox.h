#pragma once
#include <Arduino.h>
#include "physical.h"

namespace p = physical;

class Sandbox
{

public:
  void setup()
  {
    Serial.begin(9600);

    p::lcd::device.begin();
    p::car::device.begin();
  }

  void loop()
  {

    JoystickReading reading = p::joystick::device.read();
    p::lcd::device.printLine(0, "X: " + String(reading.xUnits) + " Y: " + String(reading.yUnits));
    p::car::device.tick();

    delay(100);
  }
};

namespace sandbox
{
  inline Sandbox instance;

  inline void setup() { instance.setup(); }
  inline void loop() { instance.loop(); }
}