#pragma once
#include <Arduino.h>
#include "pin.h"

class Joystick
{
public:
  Joystick(Pin &xPin, Pin &yPin, Pin &buttonPin);

  Joystick &begin();

  int readX();
  int readY();
  bool isButtonPressed();

private:
  Pin &xPin_;
  Pin &yPin_;
  Pin &buttonPin_;

  void (*pressHandlerFn_)() = nullptr;
  int debounceDelayMs_ = 50;
  volatile unsigned long lastPressTime_ = 0;

  void handleButtonInterrupt();

  friend class JoystickBuilder;
};

class JoystickBuilder
{
public:
  JoystickBuilder(PinBuilder &xPin, PinBuilder &yPin, PinBuilder &buttonPin);

  JoystickBuilder &setPressDebounce(int debounceDelayMs);
  JoystickBuilder &onPress(void (*handlerFn)());

  Joystick &build();

private:
  Pin &xPin_;
  Pin &yPin_;
  Pin &buttonPin_;
  int debounceDelayMs_ = 50;
  void (*handlerFn_)() = nullptr;
};