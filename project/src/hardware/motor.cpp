#include <Arduino.h>
#include "hardware/motor.h"

Motor::Motor(PinBuilder &powerPinBuilder, PinBuilder &directionPinBuilder) : powerPin_(powerPinBuilder.asOutput().build()),
                                                                             directionPin_(directionPinBuilder.asOutput().build())
{
}

Motor &Motor::begin()
{
  if (initialized_)
    return *this;

  powerPin_.begin();
  directionPin_.begin();

  initialized_ = true;
  return *this;
}

Motor &Motor::tick()
{
  if (!motorStateChanged_)
    return *this;

  powerPin_.analogWriteValue(map(currentPowerUnits_ * 100, 0, 100, 0, maxPower_));

  switch (currentDirection_)
  {
  case Direction::FORWARD:
    directionPin_.setLow();
    break;

  case Direction::BACKWARD:
    directionPin_.setHigh();
    break;
  }

  return *this;
}

void Motor::setPower(float powerUnit)
{
  currentDirection_ = (powerUnit >= 0) ? Direction::FORWARD : Direction::BACKWARD;
  currentPowerUnits_ = abs(powerUnit);
  motorStateChanged_ = true;
}

void Motor::stop()
{
  setPower(0);
}
