#pragma once
#include <Arduino.h>
#include "physical.h"

class Sandbox
{
public:
  void setup()
  {
    physical::car::device.begin();
  }

  void loop()
  {
    physical::car::device.tick();

    delay(100);
  }
};

namespace sandbox
{
  inline Sandbox instance;

  inline void setup() { instance.setup(); }
  inline void loop() { instance.loop(); }
}