#pragma once
#include <Arduino.h>
#include "pin.h"
#include "control/interrupt_dispatcher.h"

class Joystick
{
public:
  Joystick(PinBuilder &xPinBuilder, PinBuilder &yPinBuilder, PinBuilder &buttonPinBuilder);

  Joystick &begin();

  int readX();
  int readY();
  bool isButtonPressed();

  void onPress(Callback cb);

private:
  bool initialized_ = false;

  Pin &xPin_;
  Pin &yPin_;
  Pin &buttonPin_;
};

class JoystickBuilder
{
public:
  JoystickBuilder(PinBuilder &xPin, PinBuilder &yPin, PinBuilder &buttonPin);

  JoystickBuilder &setPressDebounce(int debounceDelayMs);
  JoystickBuilder &onPress(Callback cb);

  Joystick &build();

private:
  PinBuilder &xPinBuilder_;
  PinBuilder &yPinBuilder_;
  PinBuilder &buttonPinBuilder_;

  int debounceDelayMs_ = 0;

  std::vector<Callback> pressHandlers_;
};
