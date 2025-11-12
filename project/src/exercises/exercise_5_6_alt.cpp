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
    Angle headingsByStep[4] = {Angle(0), Angle(90), Angle(180), Angle(270)};
    float distancesCmByStep[4] = {50.0, 75.0, 100.0, 125.0};

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
  // Forward declarations
  // ------------------------------
  struct DriveState;
  struct IdleState;

  // ------------------------------
  // State class declarations only
  // ------------------------------
  struct IdleState : IState<RobotContext>
  {
    IdleState();
    static IdleState &instance();
    void onEnter(RobotContext &ctx) override;
    void tick(RobotContext &ctx) override;
    void onEvent(RobotContext &ctx, const Event &ev) override;
  };

  struct DriveState : IState<RobotContext>
  {
    DriveState();
    static DriveState &instance();
    void onEnter(RobotContext &ctx) override;
    void tick(RobotContext &ctx) override;
    void onExit(RobotContext &ctx) override;

  private:
    unsigned long startTime_ = 0;
  };

  // ------------------------------
  // Definitions (in any order now)
  // ------------------------------
  IdleState::IdleState() : IState<RobotContext>("Idle") {}
  IdleState &IdleState::instance()
  {
    static IdleState s;
    return s;
  }

  void IdleState::onEnter(RobotContext &ctx)
  {
    Serial.println(F("[Idle] Entered"));
    p::motor::leftMotor.stop();
    p::motor::rightMotor.stop();
    ctx.currentStep = 0;
  }

  void IdleState::tick(RobotContext &)
  {
    Serial.println(F("[Idle] Waiting for button press..."));
  }

  void IdleState::onEvent(RobotContext &ctx, const Event &ev)
  {
    if (ev.name() == ButtonPressedEvent::StaticName)
    {
      Serial.println(F("[Idle] Button pressed → DriveState"));
      ctx.stateMachine->transitionTo(DriveState::instance());
    }
  }

  // ------------------------------
  DriveState::DriveState() : IState<RobotContext>("Drive") {}
  DriveState &DriveState::instance()
  {
    static DriveState s;
    return s;
  }

  void DriveState::onEnter(RobotContext &ctx)
  {
    Serial.println(F("[Drive] Entered"));
    startTime_ = ctx.now();
  }

  void DriveState::tick(RobotContext &ctx)
  {
    Serial.println(F("[Drive] Driving..."));
    if (ctx.now() - startTime_ > 3000)
    {
      Serial.println(F("[Drive] Done → IdleState"));
      ctx.stateMachine->transitionTo(IdleState::instance());
    }
  }

  void DriveState::onExit(RobotContext &ctx)
  {
    p::motor::leftMotor.stop();
    p::motor::rightMotor.stop();
    Serial.println(F("[Drive] Exit"));
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