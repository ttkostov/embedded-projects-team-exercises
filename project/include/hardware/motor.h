#pragma once
#include <Arduino.h>
#include "pin.h"

class Motor
{
public:
  enum Direction
  {
    BACKWARD = 0,
    FORWARD = 1
  };

  Motor(PinBuilder &powerPin, PinBuilder &directionPin);

  Motor &begin();
  Motor &tick();

  void setPower(float powerUnits); // -1.0 to 1.0
  void stop();

private:
  bool initialized_ = false;

  Pin &powerPin_;
  Pin &directionPin_;
  bool motorStateChanged_ = true;

  const unsigned int maxPower_ = 255;

  float currentPowerUnits_ = 0; // 0 to 1.0
  Direction currentDirection_ = FORWARD;
};