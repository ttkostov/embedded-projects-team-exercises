#pragma once
#include <Arduino.h>
#include "control/state/state_machine.h"
#include "control/state/action_states.h"
#include "control/motor_driver.h"
#include "hardware/motor.h"
#include "hardware/encoder.h"
#include "hardware/compass.h"
#include "util/directional.h"
#include "physical.h"

template <typename Context>
struct CommandState : IState<Context>
{
  CommandState() : IState<Context>("Command") {}

  void onEnter(Context &ctx) override
  {
    Serial.println(F("[Command] Waiting for input..."));
  }

  void tick(Context &ctx) override
  {
    if (!Serial.available())
      return;

    // Read a full line
    String message = Serial.readStringUntil('\n');
    message.trim();

    Serial.println("[Command] Received: " + message);

    if (!handleMessage(ctx, message))
    {
      Serial.println(F("[Command] Invalid command. Try lcd:txt, dist:cm, degree:deg"));
    }
  }

  bool handleMessage(Context &ctx, String msg)
  {
    static const char *commands[] = {"lcd", "dist", "degree"};
    static const int N = sizeof(commands) / sizeof(commands[0]);

    int foundPos = -1;
    String command;

    // Find which command matches
    for (int i = 0; i < N; i++)
    {
      String c = commands[i];
      int pos = msg.indexOf(c);
      if (pos >= 0)
      {
        command = c;
        foundPos = pos;
        break;
      }
    }

    if (foundPos < 0)
      return false;

    // Extract content after the command and colon
    int colon = msg.indexOf(':', foundPos + command.length());
    if (colon < 0)
      return false;

    String content = msg.substring(colon + 1);
    content.trim();

    handleCommand(ctx, command, content);
    return true;
  }

  void handleCommand(Context &ctx, const String &command, const String &content)
  {
    p::lcd::device.clear();
    p::lcd::device.printLine(0, "Cmd: " + command);
    p::lcd::device.printLine(1, "Params: " + content);

    if (command.equalsIgnoreCase("lcd"))
    {
      Serial.println("[Command] LCD -> " + content);

      p::lcd::device.clear();
      p::lcd::device.printLine(0, content);
    }
    else if (command.equalsIgnoreCase("dist"))
    {
      float dist = content.toFloat();
      Serial.println("[Command] Drive " + String(dist) + " cm");

      ctx.stateMachine->pushState(
          *(new DriveDistanceState<Context>(dist, 0.5f)));
    }
    else if (command.equalsIgnoreCase("degree"))
    {
      float deg = content.toFloat();
      Serial.println("[Command] Turn to " + String(deg) + "°");

      ctx.stateMachine->pushState(
          *(new TurnHeadingState<Context>(Angle::absolute(deg))));
    }
  }
};