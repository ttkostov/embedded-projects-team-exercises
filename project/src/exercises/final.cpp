#include <Arduino.h>
#include <LiquidCrystal.h>
#include "hardware.h"
#include "physical.h"
#include "control/state/state_machine.h"
#include "control/state/action_states.h"
#include "control/state/command_state.h"
#include "util/directional.h"
#include <control/state/joystick_control_state.h>
#include <communication/serial_proxy.h>

namespace final
{
  namespace p = physical;

  struct RobotContext
  {
    StateMachine<RobotContext> *stateMachine = nullptr;

    // available states
    IState<RobotContext> *idleState = nullptr;
    IState<RobotContext> *commandState = nullptr;
    IState<RobotContext> *joystickState = nullptr;
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
      SerialProxy::instance().println(F("[Idle] Ready."));
    }

    void tick(RobotContext &ctx) override
    {
      p::motor::leftMotor.tick();
      p::motor::rightMotor.tick();
    }

    bool onEvent(RobotContext &ctx, const Event &ev) override
    {
      if (ev.name() == ButtonPressedEvent::StaticName)
      {
        SerialProxy::instance().println("[Idle] Button pressed -> calibrating north and transitioning to command state");
        p::compass::device.setNorthToHeading();

        ctx.stateMachine->replaceState(*ctx.joystickState);
        return true;
      }

      return false;
    }
  };

  struct ConcreteJoystickState : JoystickControlState<RobotContext>
  {
    bool onEvent(RobotContext &ctx, const Event &ev) override
    {
      if (ev.name() == ButtonPressedEvent::StaticName)
      {
        SerialProxy::instance().println("[JoystickToCommand] Button pressed -> transitioning to command state");
        ctx.stateMachine->replaceState(*ctx.commandState);
        return true;
      }

      return JoystickControlState<RobotContext>::onEvent(ctx, ev);
    }
  };

  struct ConcreteCommandState : CommandState<RobotContext>
  {
    virtual void tick(RobotContext &ctx)
    {
      String line = SerialProxy::instance().readLine();
      line.trim();
      if (line.length() > 0)
        this->onEvent(ctx, CommandReceivedEvent(line));
    };

    bool onEvent(RobotContext &ctx, const Event &ev) override
    {
      if (ev.name() == ButtonPressedEvent::StaticName)
      {
        SerialProxy::instance().println("[CommandToIdle] Button pressed -> transitioning to joystick state");
        ctx.stateMachine->replaceState(*ctx.joystickState);
        return true;
      }

      return CommandState<RobotContext>::onEvent(ctx, ev);
    }
  };

  static IdleState idleState;
  static ConcreteCommandState commandState;
  static ConcreteJoystickState joystickState;

  RobotContext ctx;
  StateMachine<RobotContext> *machine = nullptr;

  volatile bool buttonPressedFlag = false;

  void sendHeartbeat()
  {
    static unsigned long last = 0;
    unsigned long now = millis();

    if (now - last >= 5000)
    {
      last = now;
      SerialProxy::instance().println("[Heartbeat] OK");
    }
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

  void setup()
  {
    SerialProxy::instance().begin(Serial, Serial2, 9600);
    SerialProxy::instance().println("[Setup] Starting final exercise robot");

    ctx.idleState = &idleState;
    ctx.commandState = &commandState;
    ctx.joystickState = &joystickState;

    machine = new StateMachine<RobotContext>(ctx, IdleState::instance());

    p::joystick::device.onPress(makeCallback([]()
                                             { buttonPressedFlag = true; }));

    p::car::beginAll();
  }

  void loop()
  {
    if (machine)
    {
      if (buttonPressedFlag)
      {
        buttonPressedFlag = false;
        machine->handleEvent(ButtonPressedEvent());
      }

      sendHeartbeat();
      showLcdInfo();
      machine->tick();
    }

    delay(10);
  }

}