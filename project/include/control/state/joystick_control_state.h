
#pragma once
#include <Arduino.h>
#include "control/state/state_machine.h"
#include "control/state/action_states.h"
#include "hardware/compass.h"
#include "physical.h"

template <typename Context>
struct JoystickControlState : IState<Context>
{
  JoystickControlState() : IState<Context>("JoystickControl") {}

  void tick(Context &ctx) override
  {
    p::car::joystickDriver.tick();

    p::motor::leftMotor.tick();
    p::motor::rightMotor.tick();
  }

  void onExit(Context &ctx) override
  {
    p::motor::leftMotor.stop();
    p::motor::rightMotor.stop();

    p::motor::leftMotor.tick();
    p::motor::rightMotor.tick();
  }
};