#include <Arduino.h>
#include "hardware/joystick.h"

Joystick::Joystick(Pin &xPin, Pin &yPin, Pin &buttonPin)
    : xPin_(xPin), yPin_(yPin), buttonPin_(buttonPin) {}

Joystick &Joystick::begin()
{
  buttonPin_.interruptDispatcher().onFalling(
      makeCallback(this, &Joystick::handleButtonInterrupt));
  buttonPin_.interruptDispatcher().setDebounce(debounceDelayMs_);
  buttonPin_.interruptDispatcher().begin(FALLING);
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
  return buttonPin_.isLow(); // pullup active, LOW = pressed
}

void Joystick::handleButtonInterrupt()
{
  unsigned long now = millis();
  if (now - lastPressTime_ < (unsigned long)debounceDelayMs_)
  {
    return;
  }
  lastPressTime_ = now;

  if (pressHandlerFn_)
  {
    pressHandlerFn_();
  }
}