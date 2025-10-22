#pragma once
#include <Arduino.h>
#include "hardware.h"

namespace physical
{

  namespace lcd
  {
    inline PinBuilder rs(51);
    inline PinBuilder en(50);
    inline PinBuilder d4(48);
    inline PinBuilder d5(49);
    inline PinBuilder d6(47);
    inline PinBuilder d7(46);

    inline LCDDisplay &device = LCDDisplayBuilder()
                                    .withRegisterSelect(rs)
                                    .withEnable(en)
                                    .withData4(d4)
                                    .withData5(d5)
                                    .withData6(d6)
                                    .withData7(d7)
                                    .size(20, 4)
                                    .build();
  }

  namespace joystick
  {
    inline PinBuilder x(A8);
    inline PinBuilder y(A7);
    inline PinBuilder btn(18);

    inline Joystick &device = JoystickBuilder(x, y, btn)
                                  .withRotation(Joystick::Rotation::None)
                                  .withCenter(500, 493)
                                  .withMin(19, 19)
                                  .withMax(1007, 1007)
                                  // .withDeadzone(0.05)
                                  .setPressDebounce(200)
                                  .build();
  }

  namespace motor
  {
    inline PinBuilder leftPwm(9);
    inline PinBuilder leftDirection(7);

    inline PinBuilder rightPwm(10);
    inline PinBuilder rightDirection(8);

    inline PinBuilder encoderLeft(2);
    inline PinBuilder encoderRight(3);

    inline Motor leftMotor(leftPwm, leftDirection);
    inline Motor rightMotor(rightPwm, rightDirection);
  }

  namespace car
  {
    inline JoystickMotorDriver joystickDriver(motor::leftMotor, motor::rightMotor, joystick::device);
    inline Car device(joystick::device, motor::leftMotor, motor::rightMotor, joystickDriver);
  }
}