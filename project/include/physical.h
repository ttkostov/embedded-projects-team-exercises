#pragma once
#include <Arduino.h>
#include "hardware.h"

namespace physical
{

  namespace lcd
  {
    inline PinBuilder rs(37);
    inline PinBuilder en(36);
    inline PinBuilder d4(35);
    inline PinBuilder d5(34);
    inline PinBuilder d6(33);
    inline PinBuilder d7(32);

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
    inline PinBuilder x(A7);
    inline PinBuilder y(A8);
    inline PinBuilder btn(19);

    inline Joystick &device = JoystickBuilder(x, y, btn)
                                  .setPressDebounce(200)
                                  .build();
  }
}