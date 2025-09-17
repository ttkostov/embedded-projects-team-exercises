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

JoystickBuilder &JoystickBuilder::withRotation(Joystick::Rotation rotation)
{
  rotation_ = rotation;
  hasRotation_ = true;
  return *this;
}

JoystickBuilder &JoystickBuilder::withCenter(int centerX, int centerY)
{
  centerX_ = centerX;
  centerY_ = centerY;
  hasCenter_ = true;
  return *this;
}

JoystickBuilder &JoystickBuilder::withDeadzone(float deadzoneUnits)
{
  deadzoneUnits_ = deadzoneUnits;
  hasDeadzone_ = true;
  return *this;
}

JoystickBuilder &JoystickBuilder::withCurve(float curvePotential)
{
  curvePotential_ = curvePotential;
  hasDeadzone_ = true;
  return *this;
}

Joystick &JoystickBuilder::build()
{

  Joystick *js = new Joystick(
      xPinBuilder_,
      yPinBuilder_,
      buttonPinBuilder_
          .withDebounce(debounceDelayMs_));

  if (hasRotation_)
    js->rotation_ = rotation_;

  if (hasCenter_)
  {
    js->centerX_ = centerX_;
    js->centerY_ = centerY_;
  }

  if (hasDeadzone_)
    js->deadzoneUnits_ = deadzoneUnits_;

  if (hasCurve_)
    js->curvePotential_ = curvePotential_;

  // Attach all pre-configured handlers
  for (auto &cb : pressHandlers_)
  {
    js->onPress(cb);
  }

  return *js;
}
