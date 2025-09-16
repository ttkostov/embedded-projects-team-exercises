#include <Arduino.h>
#include "hardware/joystick.h"

JoystickBuilder::JoystickBuilder(PinBuilder &xPinBuilder,
                                 PinBuilder &yPinBuilder,
                                 PinBuilder &buttonPinBuilder)
    : xPin_(xPinBuilder.asInput().build()),
      yPin_(yPinBuilder.asInput().build()),
      buttonPin_(buttonPinBuilder.asInputPullup().build()) {}

JoystickBuilder &JoystickBuilder::setPressDebounce(int debounceDelayMs)
{
  debounceDelayMs_ = debounceDelayMs;
  return *this;
}

JoystickBuilder &JoystickBuilder::onPress(void (*handlerFn)())
{
  handlerFn_ = handlerFn;
  return *this;
}

Joystick &JoystickBuilder::build()
{
  static Joystick js(xPin_, yPin_, buttonPin_);
  js.debounceDelayMs_ = debounceDelayMs_;
  js.pressHandlerFn_ = handlerFn_;
  js.begin();
  return js;
}