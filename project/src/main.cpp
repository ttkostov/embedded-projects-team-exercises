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
    ex_5_6_alt
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
//   Serial.begin(9600);
//   Serial.println("Write something to the serial monitor.");
// }

// void loop()
// {
//   if (Serial.available() > 0)
//   {
//     String message = Serial.readStringUntil('\n');
//     Serial.print("Message received, content: ");
//     Serial.println(message);
//     int pos_s = message.indexOf("Print");

//     if (pos_s > -1)
//     {
//       Serial.println("Command = Print ");
//       pos_s = message.indexOf(":");

//       if (pos_s > -1)
//       {
//         String stat = message.substring(pos_s + 1);
//         if (stat == "Hi" || stat == "hi")
//         {
//           Serial.println("Hi!");
//         }
//         else if (stat == "Hello")
//         {
//           Serial.println("Hello there!");
//         }
//       }
//     }
//     else
//     {
//       Serial.println("No greeting found, try typing Print:Hi or Print:Hello\n");
//     }
//   }
// }
