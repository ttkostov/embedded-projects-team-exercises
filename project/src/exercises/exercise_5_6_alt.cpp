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

  // ------------------------------
  // Shared robot runtime context
  // ------------------------------
  struct RobotContext
  {
    StateMachine<RobotContext> *stateMachine = nullptr;

    Angle targetHeading = Angle(0);
    float targetDistanceCm = 0;

    unsigned int currentStep = 0;
    Angle headingsByStep[4] = {Angle(110), Angle(110), Angle(0)};
    float distancesCmByStep[4] = {20, 13, 20};
    float powersByStep[4] = {0.75, 0.25, 0.5};

    unsigned long now() const { return millis(); }
  };

  // ------------------------------
  // Event definitions
  // ------------------------------
  struct ButtonPressedEvent : Event
  {
    DEFINE_EVENT_TYPE("ButtonPressed");
  };

  // ------------------------------
  // State definitions
  // ------------------------------
  struct IdleState : IState<RobotContext>
  {
    IdleState() : IState<RobotContext>("Idle") {}

    static IdleState &instance()
    {
      static IdleState s;
      return s;
    }

    void onEnter(RobotContext &ctx) override
    {
      p::motor::leftMotor.stop();
      p::motor::rightMotor.stop();
      ctx.currentStep = 0;
    }

    void tick(RobotContext &ctx) override
    {
      p::motor::leftMotor.stop();
      p::motor::rightMotor.stop();
      p::motor::leftMotor.tick();
      p::motor::rightMotor.tick();

      Serial.println(F("[Idle] Waiting for events..."));
    }

    void onEvent(RobotContext &ctx, const Event &ev) override;
  };

  struct DriveState : IState<RobotContext>
  {
    DriveState() : IState<RobotContext>("Drive") {}

    static DriveState &instance()
    {
      static DriveState s;
      return s;
    }

    void onEnter(RobotContext &ctx) override
    {
      p::car::distanceDriver.reset();

      float distanceCm = ctx.distancesCmByStep[ctx.currentStep];
      float drivePower = ctx.powersByStep[ctx.currentStep];
      p::car::distanceDriver.setTarget(distanceCm, drivePower);
    }

    void onExit(RobotContext &ctx) override
    {
      p::motor::leftMotor.stop();
      p::motor::rightMotor.stop();
    }

    void tick(RobotContext &ctx) override;
  };

  // ------------------------------
  // Circularly dependant state methods
  // ------------------------------
  void IdleState::onEvent(RobotContext &ctx, const Event &ev)
  {
    if (ev.name() == ButtonPressedEvent::StaticName)
    {
      ctx.stateMachine->transitionTo(DriveState::instance());
    }
  }

  void DriveState::tick(RobotContext &ctx)
  {
    Serial.println(F("[Drive] Driving to target..."));
    p::car::distanceDriver.tick();
    if (p::car::distanceDriver.hasReachedTarget())
    {
      ctx.stateMachine->transitionTo(IdleState::instance());
    }
  }

  // ------------------------------
  // Global state
  // ------------------------------
  RobotContext ctx;
  StateMachine<RobotContext> *machine = nullptr;

  // ------------------------------
  // Arduino setup
  // ------------------------------
  void setup()
  {
    Serial.begin(9600);

    machine = new StateMachine<RobotContext>(ctx, IdleState::instance());

    p::joystick::device.onPress(makeCallback([]()
                                             {
      if (machine)
        machine->handleEvent(ButtonPressedEvent{}); }));

    p::car::beginAll();
  }

  // ------------------------------
  // Arduino loop
  // ------------------------------
  void loop()
  {
    if (machine)
      machine->tick();

    delay(200);
  }

}