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
    // ex_w4_5
    // ex_5_1
    // ex_5_2
    // ex_5_5
    // ex_5_6
    // ex_5_6_alt
    // ex_6_4
    ex_7_3
    // sandbox
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

// void setup()
// {
//   entrypoint::setup();

//   Serial.begin(9600);  // USB serial to PC
//   Serial1.begin(9600); // ESP link
// }

// void loop()
// {
//   // entrypoint::loop();

//   // From ESP to PC
//   if (Serial1.available())
//   {
//     int ch = Serial1.read();
//     Serial.write(ch);
//   }

//   // From PC to ESP
//   if (Serial.available())
//   {
//     int ch = Serial.read();
//     Serial1.write(ch);
//   }
// }
