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

// const char *commandsStr[] = {"lcd", "dist", "degree"};
// const int commandsCount = sizeof(commandsStr) / sizeof(commandsStr[0]);

// bool handleMessage(String message);
// void handleCommand(String command, String content);

// void printOnLCD(String message)
// {
//   Serial.println("LCD will print " + message);
// }

// void driveDistanceForward(int distance)
// {
//   Serial.println("The car will drive " + (String)distance + " cm forwards");
// }

// void driveDistanceBackwards(int distance)
// {
//   Serial.println("The car will drive " + (String)distance + " cm backwards");
// }
// void turnTo(int degrees)
// {
//   Serial.println("The car will turn " + (String)degrees + " degrees");
// }

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

//     bool res = handleMessage(message);
//     if (!res)
//     {
//       Serial.println("No greeting found, try typing Print:Hi or Print:Hello\n");
//     }
//   }
// }

// bool handleMessage(String message)
// {
//   int pos_s = -1;
//   String command = "";
//   for (int i = 0; i < commandsCount; i++)
//   {
//     command = String(commandsStr[i]);
//     pos_s = message.indexOf(command);
//     if (pos_s > -1)
//     {
//       // valid message found
//       break;
//     }
//   }

//   if (pos_s == -1)
//   {
//     return false;
//   }

//   String content = message.substring(pos_s + command.length() + 1);

//   Serial.println("Command = " + command);

//   handleCommand(command, content);
//   return true;
// }

// void handleCommand(String command, String content)
// {
//   if (command.equalsIgnoreCase("lcd"))
//   {
//     printOnLCD(content);
//   }
//   else if (command.equalsIgnoreCase("dist"))
//   {
//     int distance = content.toInt();
//     if (distance < 0)
//     {
//       driveDistanceBackwards(abs(distance));
//     }
//     else
//     {
//       driveDistanceForward(abs(distance));
//     }
//   }
//   else if (command.equalsIgnoreCase("degree"))
//   {
//     int degree = content.toInt();
//     turnTo(degree);
//   }
// }

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
