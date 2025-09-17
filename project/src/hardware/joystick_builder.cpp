#include "hardware/joystick.h"

JoystickBuilder::JoystickBuilder(PinBuilder &xPin, PinBuilder &yPin, PinBuilder &buttonPin)
    : xPinBuilder_(xPin), yPinBuilder_(yPin), buttonPinBuilder_(buttonPin) {}

JoystickBuilder &JoystickBuilder::setPressDebounce(int debounceDelayMs)
{
  debounceDelayMs_ = debounceDelayMs;
  return *this;
}

JoystickBuilder &JoystickBuilder::onPress(Callback cb)
{
  pressHandlers_.push_back(cb);
  return *this;
}

Joystick &JoystickBuilder::build()
{

  Joystick *js = new Joystick(
      xPinBuilder_,
      yPinBuilder_,
      buttonPinBuilder_
          .withDebounce(debounceDelayMs_));

  // Attach all pre-configured handlers
  for (auto &cb : pressHandlers_)
  {
    js->onPress(cb);
  }

  return *js;
}
