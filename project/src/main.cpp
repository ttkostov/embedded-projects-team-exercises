#include <Arduino.h>
#include "exercises.h"
#include "hardware.h"

namespace entrypoint =
    // ex_w1
    ex_w2_1_2
    // Activate the exercise you want to run by uncommenting it
    ;

PinBuilder joystickXBuilder(34);
PinBuilder joystickYBuilder(35);
PinBuilder joystickButtonBuilder(32);

JoystickBuilder joystickBuilder(PinBuilder(32), joystickYBuilder, joystickButtonBuilder);
Joystick &joystick = joystickBuilder.build();

void setup()
{
  entrypoint::setup();
}

void loop()
{
  entrypoint::loop();
}
