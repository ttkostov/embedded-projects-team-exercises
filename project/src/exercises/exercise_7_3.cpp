#include <Arduino.h>
#include <LiquidCrystal.h>
#include "hardware.h"
#include "physical.h"
#include "control/state/state_machine.h"
#include "control/state/action_states.h"
#include "control/state/command_state.h"
#include "util/directional.h"

namespace ex_7_3
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
        Serial.println(F("[Idle] Button pressed -> transitioning to command state"));
        ctx.stateMachine->pushState(TriggeredState());
      }
    }

    static IState<RobotContext> &TriggeredState();
  };

  IState<RobotContext> &IdleState::TriggeredState()
  {
    static CommandState<RobotContext> initialState;

    return initialState;
  }

  RobotContext ctx;
  StateMachine<RobotContext> *machine = nullptr;

  void setup()
  {
    Serial.begin(9600);
    Serial1.begin(9600);

    machine = new StateMachine<RobotContext>(ctx, IdleState::instance());

    p::joystick::device.onPress(makeCallback([]()
                                             {
        if (machine)
          machine->handleEvent(ButtonPressedEvent{}); }));

    p::car::beginAll();
  }

  void loop()
  {
    if (machine)
    {

      if (Serial.available())
      {
        String line = Serial.readStringUntil('\n');
        line.trim();
        if (line.length() > 0)
          machine->handleEvent(CommandReceivedEvent(line));
      }

      if (Serial1.available())
      {
        String line = Serial1.readStringUntil('\n');
        line.trim();
        if (line.length() > 0)
          machine->handleEvent(CommandReceivedEvent(line));
      }

      machine->tick();
    }

    delay(10);
  }

}