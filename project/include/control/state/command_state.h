#pragma once
#include <Arduino.h>
#include "control/state/state_machine.h"
#include "control/state/action_states.h"
#include "communication/serial_proxy.h"
#include "control/command.h"
#include "control/motor_driver.h"
#include "hardware/motor.h"
#include "hardware/encoder.h"
#include "hardware/compass.h"
#include "util/directional.h"
#include "physical.h"

struct CommandReceivedEvent : Event
{
  DEFINE_EVENT_TYPE("CommandReceived");
  String text;

  CommandReceivedEvent(const String &t) : text(t) {}
};

template <typename Context>
struct CommandState : IState<Context>
{
  CommandState() : IState<Context>("Command") {}

  CommandParser parser;

  void onEnter(Context &ctx) override
  {
    SerialProxy::instance().println("[Command] Waiting for input...");
  }

  bool onEvent(Context &ctx, const Event &ev) override
  {
    if (ev.name() != CommandReceivedEvent::StaticName)
      return false;

    auto &cmdEv = static_cast<const CommandReceivedEvent &>(ev);

    String msg = cmdEv.text;
    msg.trim();
    if (msg.length() == 0)
      return true;

    SerialProxy::instance().println("[Command] Received: " + msg);

    auto parsed = parser.parse(msg);

    if (parsed.command.length() == 0)
    {
      SerialProxy::instance().println("[Command] Invalid command.");
      return true;
    }

    handleCommand(ctx, parsed);
    return true;
  }

  void handleCommand(Context &ctx, const CommandParseResult &cmd)
  {
    String c = cmd.command;

    if (c.equalsIgnoreCase("lcd"))
    {
      handleLCD(cmd);
    }
    else if (c.equalsIgnoreCase("move"))
    {
      handleMove(ctx, cmd);
    }
    else if (c.equalsIgnoreCase("turn"))
    {
      handleTurn(ctx, cmd);
    }
    else if (c.equalsIgnoreCase("find"))
    {
      handleFind(ctx, cmd);
    }
    else
    {
      SerialProxy::instance().println("[Command] Unknown command: " + c);
    }
  }

  void handleLCD(const CommandParseResult &cmd)
  {
    SerialProxy::instance().println("[LCD] " + cmd.rawValue);
    p::lcd::device.clear();
    p::lcd::device.printLine(0, cmd.rawValue);
  }

  void handleMove(Context &ctx, const CommandParseResult &cmd)
  {
    float dist = cmd.numericValue;
    float speed = cmd.getParamFloat("speed", 0.5f);

    auto *state = new DriveDistanceState<Context>(dist, speed);
    ctx.stateMachine->pushState(*state);
  }

  void handleTurn(Context &ctx, const CommandParseResult &cmd)
  {
    float deg = cmd.numericValue;
    bool relative = cmd.hasFlag("--relative");

    Angle target = relative ? Angle::relative(deg)
                            : Angle::absolute(deg);

    SerialProxy::instance().print("[Degree] Turning ");
    SerialProxy::instance().print(relative ? "(relative) " : "(absolute) ");
    SerialProxy::instance().println(String(deg) + "°");

    auto *state = new TurnHeadingState<Context>(target);
    ctx.stateMachine->pushState(*state);
  }

  void handleFind(Context &ctx, const CommandParseResult &cmd)
  {
    Angle target = Angle::absolute(0);

    SerialProxy::instance().print("[Find] Finding north (0°) ");

    auto *state = new TurnHeadingState<Context>(target);
    ctx.stateMachine->pushState(*state);
  }
};
