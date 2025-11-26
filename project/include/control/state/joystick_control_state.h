
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

    showLcdInfo();
  }

  void onExit(Context &ctx) override
  {
    p::motor::leftMotor.stop();
    p::motor::rightMotor.stop();

    p::motor::leftMotor.tick();
    p::motor::rightMotor.tick();
  }

  void showLcdInfo()
  {
    CompassReading headingReading = p::compass::device.read();
    p::lcd::device.printLine(0, "Heading: " + String(headingReading.headingDirection.toString()) + " (" + String(headingReading.heading.value(), 1) + ")");

    int leftEncodings = p::motor::leftEncoder.getTicks();
    float leftDistance = p::motor::leftEncoder.getDistanceCm();

    int rightEncodings = p::motor::rightEncoder.getTicks();
    float rightDistance = p::motor::rightEncoder.getDistanceCm();

    p::lcd::device.printLine(1, "L: " + String(leftEncodings) + " (" + String(leftDistance, 1) + "cm)");
    p::lcd::device.printLine(2, "R: " + String(rightEncodings) + " (" + String(rightDistance, 1) + "cm)");
  }
};