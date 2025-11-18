#include <Arduino.h>
#include <LiquidCrystal.h>
#include "hardware.h"
#include "physical.h"
#include "control/state/state_machine.h"
#include "control/state/action_states.h"
#include "util/directional.h"

namespace ex_5_6_alt
{
  namespace p = physical;

  struct RobotContext
  {
    StateMachine<RobotContext> *stateMachine = nullptr;
  };

  struct ButtonPressedEvent : Event
  {
    DEFINE_EVENT_TYPE("ButtonPressed");
  };

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
      Serial.println(F("[Idle] Ready."));
    }

    void tick(RobotContext &ctx) override
    {
      p::motor::leftMotor.tick();
      p::motor::rightMotor.tick();
    }

    void onEvent(RobotContext &ctx, const Event &ev) override
    {
      if (ev.name() == ButtonPressedEvent::StaticName)
      {
        Serial.println(F("[Idle] Button pressed → starting triangle"));
        ctx.stateMachine->pushState(SequenceStateInstance());
      }
    }

    static IState<RobotContext> &SequenceStateInstance();
  };

  IState<RobotContext> &IdleState::SequenceStateInstance()
  {
    static DriveDistanceState<RobotContext> leg1(20, 0.75f);
    static TurnHeadingState<RobotContext> turn1(Angle(110));

    static DriveDistanceState<RobotContext> leg2(13, 0.25f);
    static TurnHeadingState<RobotContext> turn2(Angle(110));

    static DriveDistanceState<RobotContext> leg3(20, 0.50f);
    static TurnHeadingState<RobotContext> turn3(Angle(0));

    static SequenceState<RobotContext> seq{
        &leg1, &turn1,
        &leg2, &turn2,
        &leg3, &turn3};

    return seq;
  }

  RobotContext ctx;
  StateMachine<RobotContext> *machine = nullptr;

  void setup()
  {
    Serial.begin(9600);

    p::car::beginAll();

    machine = new StateMachine<RobotContext>(ctx, IdleState::instance());

    p::joystick::device.onPress(makeCallback([]()
                                             {
        if (machine)
          machine->handleEvent(ButtonPressedEvent{}); }));
  }

  void loop()
  {
    if (machine)
      machine->tick();

    delay(10);
  }

}