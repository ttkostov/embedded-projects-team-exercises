#include <Arduino.h>
#include <algorithm>
#include <LiquidCrystal.h>
#include "exercises.h"
#include "hardware.h"
#include "physical.h"
#include <Wire.h>
#include "control/state_machine.h"

namespace ex_5_6_alt
{
  namespace p = physical;

  struct RobotContext
  {
    // Pointer to the owning state machine (injected automatically)
    StateMachine<RobotContext> *stateMachine = nullptr;

    // Shared runtime data
    Angle targetHeading = Angle(0);

    unsigned long now() const { return millis(); }
  };

  struct IdleState : IState<RobotContext>
  {
    static IdleState &instance()
    {
      static IdleState s;
      return s;
    }

    void onEnter(RobotContext &ctx) override
    {
      Serial.println(F("[Idle] Enter"));
      p::motor::leftMotor.stop();
      p::motor::rightMotor.stop();
    }

    void tick(RobotContext &ctx) override
    {
      // Example: check joystick or serial
      if (ctx.joystick.isButtonPressed())
      {
        Serial.println(F("[Idle] Joystick pressed, start driving"));
        ctx.stateMachine->transitionTo(DriveState::instance());
      }
    }
  };

  struct DriveState : IState<RobotContext>
  {
    static DriveState &instance()
    {
      static DriveState s;
      return s;
    }

    void onEnter(RobotContext &ctx) override
    {
      Serial.println(F("[Drive] Enter"));
      ctx.leftMotor.setPower(0.5);
      ctx.rightMotor.setPower(0.5);
      startTime_ = ctx.now();
    }

    void tick(RobotContext &ctx) override
    {
      // drive for 3 seconds
      if (ctx.now() - startTime_ > 3000)
      {
        Serial.println(F("[Drive] Done driving"));
        ctx.stateMachine->transitionTo(IdleState::instance());
      }
    }

    void onExit(RobotContext &ctx) override
    {
      Serial.println(F("[Drive] Exit"));
      ctx.leftMotor.stop();
      ctx.rightMotor.stop();
    }

  private:
    unsigned long startTime_ = 0;
  };

  void setup()
  {
    // put your setup code here, to run once:
    Serial.begin(9600);

    p::joystick::device.onPress(makeCallback(onButtonPressed));
    p::joystick::device.begin();

    p::motor::encoderLeft
        .asInputPullup()
        .withChangeInterrupt(
            makeCallback(
                []()
                {
                  leftCounter++;

                  if (distanceDriven == false && leftCounter >= targetEncodings)
                  {
                    stopMotor();
                    distanceDriven = true;
                  }
                }))
        .build()
        .begin();

    p::motor::encoderRight.asInputPullup().withChangeInterrupt(makeCallback(
                                                                   []()
                                                                   {
    if(countEncodings) rightCounter++; }))
        .build()
        .begin();

    p::lcd::device.begin();

    Wire.begin();
  }

  void loop()
  {
    float heading = offsetDegrees(readHeadingDegrees(), 180);

    String direction = getHeadingDirection(heading);
    p::lcd::device.printLine(2, "Heading " + direction + " :" + String(heading, 2));

    executeProgramState();

    delay(200);
  }
}