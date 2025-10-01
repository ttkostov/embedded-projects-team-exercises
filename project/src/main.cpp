#include <Arduino.h>
#include <exercises.h>
#include <sandbox.h>

namespace entrypoint =
    // ex_w1
    // ex_w2_1_2
    // ex_w2_1_2_alt
    // ex_w3_3_4
    // ex_w3_5_6
    // ex_w4_4
    sandbox
    // Activate the exercise you want to run by uncommenting it
    ;

void setup()
{
  entrypoint::setup();
}

void loop()
{
  entrypoint::loop();
}
