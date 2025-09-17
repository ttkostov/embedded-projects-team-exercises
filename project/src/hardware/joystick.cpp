#include <Arduino.h>
#include "hardware/joystick.h"

Joystick::Joystick(PinBuilder &xPinBuilder, PinBuilder &yPinBuilder, PinBuilder &buttonPinBuilder)
    : xPin_(xPinBuilder.asInput().build()),
      yPin_(yPinBuilder.asInput().build()),
      buttonPin_(buttonPinBuilder.asInputPullup().build())
{
}

Joystick &Joystick::begin()
{
  if (initialized_)
    return *this;

  xPin_.begin();
  yPin_.begin();
  buttonPin_.begin();

  initialized_ = true;
  return *this;
}

int Joystick::readX()
{
  return xPin_.analogReadValue();
}

int Joystick::readY()
{
  return yPin_.analogReadValue();
}

bool Joystick::isButtonPressed()
{
  return buttonPin_.isLow(); // pull-up active, LOW = pressed
}

void Joystick::onPress(Callback cb)
{
  buttonPin_.interruptDispatcher().onChange(cb);
}
