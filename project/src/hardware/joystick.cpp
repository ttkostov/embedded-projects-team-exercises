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

JoystickReading Joystick::read()
{
  int rawX = xPin_.analogReadValue();
  int rawY = yPin_.analogReadValue();

  // normalize
  float normX = normalizeAxis(rawX, centerX_, minX_, maxX_);
  float normY = -normalizeAxis(rawY, centerY_, minY_, maxY_); // Negate Y to have up = positive

  // rotate, clamp, correct deadzone, curve shaping
  applyRotation(normX, normY);
  clampToUnit(normX, normY);
  bool isIdle = applyDeadzone(normX, normY);

  if (!isIdle)
    applyCurve(normX, normY);

  // cache values for consistent partial reads
  lastX_ = normX;
  lastY_ = normY;
  lastIdle_ = isIdle;

  return JoystickReading{normX, normY, isIdle};
}

void Joystick::applyRotation(float &x, float &y)
{
  float oldX = x, oldY = y;

  switch (rotation_)
  {
  case Rotation::None:
    // no change
    break;
  case Rotation::CW90:
    x = oldY;
    y = -oldX;
    break;
  case Rotation::CW180:
    x = -oldX;
    y = -oldY;
    break;
  case Rotation::CW270:
    x = -oldY;
    y = oldX;
    break;
  }
}

void Joystick::clampToUnit(float &x, float &y)
{
  x = constrain(x, -1.0f, 1.0f);
  y = constrain(y, -1.0f, 1.0f);
}

bool Joystick::applyDeadzone(float &x, float &y)
{
  float magnitude = sqrt(x * x + y * y);
  if (magnitude < deadzoneUnits_)
  {
    x = 0;
    y = 0;
    return true; // joystick idle
  }
  return false;
}

void Joystick::applyCurve(float &x, float &y)
{
  if (curvePotential_ != 1.0f)
  {
    x = copysign(pow(fabs(x), curvePotential_), x);
    y = copysign(pow(fabs(y), curvePotential_), y);
  }
}

bool Joystick::isButtonPressed()
{
  return buttonPin_.isLow(); // pull-up active, LOW = pressed
}

void Joystick::onPress(Callback cb)
{
  buttonPin_.interruptDispatcher().onChange(cb);
}

float Joystick::normalizeAxis(int raw, int center, int min, int max)
{
  if (raw >= center)
    // Right/up side
    return (raw - center) / float(max - center);

  // Left/down side
  return (raw - center) / float(center - min);
}

// ----------------- JoystickBuilder ----------------

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

JoystickBuilder &JoystickBuilder::withMax(int maxX, int maxY)
{
  maxX_ = maxX;
  maxY_ = maxY;
  hasMax_ = true;
  return *this;
}

JoystickBuilder &JoystickBuilder::withMin(int minX, int minY)
{
  minX_ = minX;
  minY_ = minY;
  hasMin_ = true;
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

  if (hasMax_)
  {
    js->maxX_ = maxX_;
    js->maxY_ = maxY_;
  }

  if (hasMin_)
  {
    js->minX_ = minX_;
    js->minY_ = minY_;
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
