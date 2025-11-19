#pragma once
#include <Arduino.h>
#include "control/state/state_machine.h"
#include "control/state/action_states.h"
#include "control/command.h"
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

  CommandParser parser;

  void onEnter(Context &ctx) override
  {
    Serial.println("[Command] Waiting for input...");
  }

  void tick(Context &ctx) override
  {
    if (!Serial.available())
      return;

    String msg = Serial.readStringUntil('\n');
    msg.trim();

    if (msg.length() == 0)
      return;

    Serial.println("[Command] Received: " + msg);

    auto parsed = parser.parse(msg);

    if (parsed.command.length() == 0)
    {
      Serial.println("[Command] Invalid command.");
      return;
    }

    handleCommand(ctx, parsed);
  }

  void handleCommand(Context &ctx, const CommandParseResult &cmd)
  {
    String c = cmd.command;

    p::lcd::device.clear();
    p::lcd::device.printLine(0, "Cmd: " + c);
    p::lcd::device.printLine(1, "Val: " + cmd.rawValue);

    if (c.equalsIgnoreCase("lcd"))
    {
      handleLCD(cmd);
    }
    else if (c.equalsIgnoreCase("dist"))
    {
      handleDist(ctx, cmd);
    }
    else if (c.equalsIgnoreCase("degree"))
    {
      handleDegree(ctx, cmd);
    }
    else
    {
      Serial.println("[Command] Unknown command: " + c);
    }
  }

  void handleLCD(const CommandParseResult &cmd)
  {
    Serial.println("[LCD] " + cmd.rawValue);
    p::lcd::device.clear();
    p::lcd::device.printLine(0, cmd.rawValue);
  }

  void handleDist(Context &ctx, const CommandParseResult &cmd)
  {
    float dist = cmd.numericValue;
    float speed = cmd.getParamFloat("speed", 0.5f);

    auto *state = new DriveDistanceState<Context>(dist, speed);
    ctx.stateMachine->pushState(*state);
  }

  void handleDegree(Context &ctx, const CommandParseResult &cmd)
  {
    float deg = cmd.numericValue;
    bool relative = cmd.hasFlag("--relative");

    Angle target = relative ? Angle::relative(deg)
                            : Angle::absolute(deg);

    Serial.print("[Degree] Turning ");
    Serial.print(relative ? "(relative) " : "(absolute) ");
    Serial.println(String(deg) + "°");

    auto *state = new TurnHeadingState<Context>(target);
    ctx.stateMachine->pushState(*state);
  }
};
