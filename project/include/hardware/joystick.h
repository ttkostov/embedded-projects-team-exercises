#pragma once
#include <Arduino.h>
#include "pin.h"
#include "control/interrupt_dispatcher.h"

struct JoystickReading
{
  float xUnits; // -1.0 to 1.0
  float yUnits; // -1.0 to 1.0
  bool isIdle;
};

class Joystick
{
public:
  enum class Rotation
  {
    None,
    CW90,
    CW180,
    CW270
  };

  Joystick(PinBuilder &xPinBuilder, PinBuilder &yPinBuilder, PinBuilder &buttonPinBuilder);

  Joystick &begin();

  JoystickReading read();
  bool isButtonPressed();

  void onPress(Callback cb);

private:
  friend class JoystickBuilder;

  bool initialized_ = false;

  Pin &xPin_;
  Pin &yPin_;
  Pin &buttonPin_;

  const int maxX_ = 1023;
  const int maxY_ = 1023;

  Rotation rotation_ = Rotation::None;
  int centerX_ = 512;
  int centerY_ = 512;
  float deadzoneUnits_ = 0.05; // 5%
  float curvePotential_ = 1.0; // 1.0 = linear

  // Cached values
  float lastX_ = 0;
  float lastY_ = 0;
  bool lastIdle_ = true;

  void applyRotation(float &x, float &y);
  void clampToUnit(float &x, float &y);
  bool applyDeadzone(float &x, float &y);
  void applyCurve(float &x, float &y);
  float normalizeAxis(int raw, int center, int min, int max);
};

class JoystickBuilder
{
public:
  JoystickBuilder(PinBuilder &xPin, PinBuilder &yPin, PinBuilder &buttonPin);

  JoystickBuilder &setPressDebounce(int debounceDelayMs);
  JoystickBuilder &onPress(Callback cb);

  JoystickBuilder &withRotation(Joystick::Rotation rotation);
  JoystickBuilder &withCenter(int centerX, int centerY);
  JoystickBuilder &withDeadzone(float deadzoneUnits);
  JoystickBuilder &withCurve(float curvePotential);

  Joystick &build();

private:
  PinBuilder &xPinBuilder_;
  PinBuilder &yPinBuilder_;
  PinBuilder &buttonPinBuilder_;

  int debounceDelayMs_ = 0;

  Joystick::Rotation rotation_ = Joystick::Rotation::None;
  bool hasRotation_ = false;

  int centerX_;
  int centerY_;
  bool hasCenter_ = false;

  float deadzoneUnits_;
  bool hasDeadzone_ = false;

  float curvePotential_;
  bool hasCurve_ = false;

  std::vector<Callback> pressHandlers_;

  float normalizeAxis(int raw, int center, int min, int max);
};
