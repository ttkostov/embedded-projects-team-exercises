// === Combined C++ Submission ===
// Generated on Wed Dec 10 12:10:24 AM EET 2025
// Source roots: ./src ./include

// === Header Files ===

// ===== File: ./include/communication/i2c.h =====
#pragma once
#include <Arduino.h>

class I2CBus
{
public:
  // Singleton access
  static I2CBus &instance()
  {
    static I2CBus instance;
    return instance;
  }

  I2CBus &begin();

  byte readByte(uint8_t address, uint8_t reg);

private:
  I2CBus() {} // Hide constructor for singleton pattern

  bool initialized_ = false;
};
// ===== File: ./include/communication/serial_proxy.h =====
#pragma once
#include <Arduino.h>

class SerialProxy
{
public:
  static SerialProxy &instance()
  {
    static SerialProxy inst;
    return inst;
  }

  void begin(HardwareSerial &usbPort, HardwareSerial &espPort, unsigned long baud = 9600)
  {
    usb = &usbPort;
    esp = &espPort;
    baud_ = baud;

    usb->begin(baud_);
    esp->begin(baud_);
  }

  void print(const String &s)
  {
    if (forwardToUSB && usb)
      usb->print(s);
    if (forwardToESP && esp)
      esp->print(s);
  }

  void println(const String &s)
  {
    if (forwardToUSB && usb)
      usb->println(s);
    if (forwardToESP && esp)
      esp->println(s);
  }

  String readLine()
  {
    // First check USB
    if (usb && usb->available())
      return usb->readStringUntil('\n');

    // Then ESP
    if (esp && esp->available())
      return esp->readStringUntil('\n');

    return "";
  }

  void setForwardUSB(bool v) { forwardToUSB = v; }
  void setForwardESP(bool v) { forwardToESP = v; }

private:
  SerialProxy() {}

  HardwareSerial *usb = nullptr;
  HardwareSerial *esp = nullptr;

  bool forwardToUSB = true;
  bool forwardToESP = true;
  unsigned long baud_;
};

// ===== File: ./include/control/callback.h =====
#pragma once
#include <Arduino.h>

struct Callback
{
  using Thunk = void (*)(void *);

  void *ctx = nullptr; // object pointer or custom data
  Thunk fn = nullptr;  // trampoline

  void operator()() const
  {
    if (fn)
      fn(ctx);
  }
};

// Free/static function binder
inline Callback makeCallback(void (*fn)())
{
  struct Data
  {
    void (*func)();
  };
  auto *data = new Data{fn};

  return {
      data,
      [](void *ctx)
      {
        auto *d = static_cast<Data *>(ctx);
        d->func();
      }};
}

// Member function binder
template <typename T>
inline Callback makeCallback(T *obj, void (T::*method)())
{
  struct Data
  {
    T *obj;
    void (T::*method)();
  };
  auto *data = new Data{obj, method}; // allocate once, never freed

  return {
      data,
      [](void *ctx)
      {
        auto *d = static_cast<Data *>(ctx);
        (d->obj->*d->method)();
      }};
}

// ===== File: ./include/control/command.h =====
#pragma once
#include <Arduino.h>

struct CommandParseResult
{
  static constexpr uint8_t MAX_FLAGS = 6;
  static constexpr uint8_t MAX_PARAMS = 6;

  String command;         // e.g. "list"
  String rawValue;        // e.g. "/tes/path"
  float numericValue = 0; // parsed number if present

  String flags[MAX_FLAGS]; // e.g. "--recursive"
  uint8_t flagCount = 0;

  String paramKeys[MAX_PARAMS];   // e.g. "key"
  String paramValues[MAX_PARAMS]; // e.g. "value"
  uint8_t paramCount = 0;

  bool hasFlag(const char *flag) const
  {
    for (uint8_t i = 0; i < flagCount; i++)
      if (flags[i].equalsIgnoreCase(flag))
        return true;
    return false;
  }

  bool hasParam(const char *key) const
  {
    for (uint8_t i = 0; i < paramCount; i++)
      if (paramKeys[i].equalsIgnoreCase(key))
        return true;
    return false;
  }

  float getParamFloat(const char *key, float defaultVal = 0) const
  {
    for (uint8_t i = 0; i < paramCount; i++)
      if (paramKeys[i].equalsIgnoreCase(key))
        return paramValues[i].toFloat();
    return defaultVal;
  }
};

class CommandParser
{
public:
  CommandParseResult parse(const String &msg)
  {
    CommandParseResult result;

    int colon = msg.indexOf(':');
    if (colon < 0)
      return result;

    result.command = msg.substring(0, colon);
    String remainder = msg.substring(colon + 1);
    remainder.trim();

    tokenize(result, remainder);
    extractNumber(result);

    return result;
  }

private:
  void tokenize(CommandParseResult &out, const String &input)
  {
    unsigned int start = 0;

    while (start < input.length())
    {
      int end = input.indexOf(' ', start);
      if (end < 0)
        end = input.length();

      String tok = input.substring(start, end);
      tok.trim();

      if (tok.length() > 0)
        processToken(out, tok);

      start = end + 1;
    }
  }

  void processToken(CommandParseResult &out, const String &tok)
  {
    // Parameter: --key=value
    if (tok.startsWith("--") && tok.indexOf('=') > 0)
    {
      int eq = tok.indexOf('=');
      if (out.paramCount < out.MAX_PARAMS)
      {
        out.paramKeys[out.paramCount] = tok.substring(2, eq);
        out.paramValues[out.paramCount] = tok.substring(eq + 1);
        out.paramCount++;
      }
      return;
    }

    // Flag: --option
    if (tok.startsWith("--"))
    {
      if (out.flagCount < out.MAX_FLAGS)
        out.flags[out.flagCount++] = tok;
      return;
    }

    // Otherwise: it's the main value
    if (out.rawValue.length() == 0)
      out.rawValue = tok;
  }

  void extractNumber(CommandParseResult &out)
  {
    if (out.rawValue.length() > 0)
      out.numericValue = out.rawValue.toFloat();
  }
};

// ===== File: ./include/control/interrupt_dispatcher.h =====
#pragma once
#include <Arduino.h>
#include <vector>
#include "control/callback.h"

class InterruptDispatcher
{
public:
  explicit InterruptDispatcher(uint8_t id, int lastState = LOW);

  void begin(); // registers interrupt
  void handleInterrupt();

  void setDebounce(unsigned long ms);

  void onLow(Callback cb);
  void onHigh(Callback cb);
  void onChange(Callback cb);
  void onRising(Callback cb);
  void onFalling(Callback cb);

  bool hasHandlers() const;

  static InterruptDispatcher *dispatchers_[NUM_DIGITAL_PINS];

private:
  bool initialized_ = false;

  uint8_t pinId_;
  unsigned long debounceMs_ = 0;
  volatile unsigned long lastInterruptTime_ = 0;
  volatile int lastState_ = LOW;

  std::vector<Callback> lowHandlers_;
  std::vector<Callback> highHandlers_;
  std::vector<Callback> changeHandlers_;
  std::vector<Callback> risingHandlers_;
  std::vector<Callback> fallingHandlers_;

  static inline int fastRead(uint8_t pin);
};

/*
  This trampoline is needed to create a unique static handler for each pin.
  This is because attachInterrupt requires a static function pointer.
  This means we cannot use a member function directly.
*/
template <int N>
struct Trampoline
{
  static void handler()
  {
    if (InterruptDispatcher::dispatchers_[N])
    {
      InterruptDispatcher::dispatchers_[N]->handleInterrupt();
    }
  }
};

// ===== File: ./include/control/motor_driver.h =====
#pragma once
#include <Arduino.h>
#include "hardware/motor.h"
#include "hardware/joystick.h"
#include "hardware/compass.h"
#include "util/directional.h"
#include "hardware/encoder.h"

class IMotorDriver
{
public:
  virtual void begin() = 0;
  virtual void tick() = 0;
};

class JoystickMotorDriver : public IMotorDriver
{
public:
  JoystickMotorDriver(Motor &leftMotor, Motor &rightMotor, Joystick &joystick);

  void begin() override;
  void tick() override;

private:
  bool initialized_ = false;

  Motor &leftMotor_;
  Motor &rightMotor_;

  Joystick &joystick_;
};

class JoystickVectorMotorDriver : public IMotorDriver
{
public:
  JoystickVectorMotorDriver(Motor &leftMotor, Motor &rightMotor, Joystick &joystick);

  void begin() override;
  void tick() override;

private:
  bool initialized_ = false;

  Motor &leftMotor_;
  Motor &rightMotor_;

  Joystick &joystick_;
};

class CompassHeadingMotorDriver : public IMotorDriver
{
public:
  CompassHeadingMotorDriver(Motor &leftMotor, Motor &rightMotor, Compass &compass);

  void begin() override;
  void tick() override;

  void setTolerance(const Angle &tolerance);
  void setTargetHeading(const Angle &heading);
  bool hasReachedTarget() const;

  void reset();

private:
  bool initialized_ = false;

  Motor &leftMotor_;
  Motor &rightMotor_;
  Compass &compass_;

  Angle targetHeading_;
  Angle tolerance_ = Angle(3.0);

  bool targetSet_ = false;
  bool reachedTarget_ = false;

  bool approachTarget();
  void stopMotors();
};

// Drives both motors straight based on a single encoder measurement.
class DistanceMotorDriver : public IMotorDriver
{
public:
  DistanceMotorDriver(Motor &leftMotor, Motor &rightMotor, Encoder &encoder);

  void begin() override;
  void tick() override;

  // Configure a target distance (in cm) and power (-1.0 - 1.0)
  void setTarget(float distanceCm, float power = 0.5f);
  bool hasReachedTarget() const;

  void reset();

  void onEncoderTick();

private:
  Motor &leftMotor_;
  Motor &rightMotor_;
  Encoder &encoder_;

  bool initialized_ = false;
  bool targetSet_ = false;
  bool reachedTarget_ = false;

  float targetDistanceCm_ = 0.0f;
  float startDistanceCm_ = 0.0f;
  float drivePower_ = 0.5f;

  void driveStraight();
  void stopMotors();
};
// ===== File: ./include/control/state/action_states.h =====
#pragma once
#include <Arduino.h>
#include "control/state/state_machine.h"
#include "control/motor_driver.h"
#include "hardware/motor.h"
#include "hardware/encoder.h"
#include "hardware/compass.h"
#include "util/directional.h"
#include "physical.h"

// ---------------------------------------------------------------------------
// Base class for reusable action states
//
// These are states that:
//  - perform a finite action (drive distance, turn, wait for delay, …)
//  - automatically return to the state from which they were activated
// ---------------------------------------------------------------------------

namespace p = physical;

// ActionState base class
//
// - Performs a finite action
// - Automatically returns to the previous state when finished
//
template <typename Context>
struct ActionState : IState<Context>
{
  ActionState(const char *name) : IState<Context>(name) {}

  void finish(Context &ctx)
  {
    if (ctx.stateMachine)
      ctx.stateMachine->popState(); // pop this state
  }
};

// Executes a sequence of states in order.
// When all steps are complete, returns to the previous state.
//
// Requirements for Context:
// - All states in the sequence must be compatible with Context.
//
template <typename Context, uint8_t MaxSteps = 8>
struct SequenceState : ActionState<Context>
{
  IState<Context> *steps[MaxSteps];
  uint8_t stepCount = 0;
  uint8_t current = 0;

  SequenceState(const char *name, std::initializer_list<IState<Context> *> list)
      : ActionState<Context>(name)
  {
    for (auto *s : list)
    {
      if (stepCount < MaxSteps)
        steps[stepCount++] = s;
    }
  }

  void onEnter(Context &ctx) override
  {
    // First time entering the sequence
    ActionState<Context>::onEnter(ctx);

    if (stepCount == 0)
    {
      this->finish(ctx);
      return;
    }

    current = 0;
    ctx.stateMachine->pushState(*steps[current]);
  }

  void onResume(Context &ctx) override
  {
    // A child step just completed (was popped)
    current++;

    if (current >= stepCount)
    {
      this->finish(ctx);
      return;
    }

    // push the next step
    ctx.stateMachine->pushState(*steps[current]);
  }

  void tick(Context &ctx) override
  {
    // SequenceState itself does nothing during tick.
    // Child step is active and receives ticks.
  }
};

// Drive a fixed distance in centimeters
//
// Requirements for Context: none
//
template <typename Context>
struct DriveDistanceState : ActionState<Context>
{
  float distanceCm;
  float power;

  DriveDistanceState(float dist, float pwr = 0.5f)
      : ActionState<Context>("DriveDistance"),
        distanceCm(dist),
        power(pwr)
  {
  }

  void onEnter(Context &ctx) override
  {
    ActionState<Context>::onEnter(ctx);

    p::car::distanceDriver.setTarget(abs(distanceCm), distanceCm > 0 ? power : -power);
  }

  void tick(Context &ctx) override
  {
    p::car::distanceDriver.tick();
    if (p::car::distanceDriver.hasReachedTarget())
      this->finish(ctx);
  }
};

// Turn to a specific heading (in degrees)
//
// Requirements for Context: none
//
template <typename Context>
struct TurnHeadingState : ActionState<Context>
{
  Angle target;

  TurnHeadingState(const Angle &targetHeading)
      : ActionState<Context>("TurnHeading"),
        target(targetHeading)
  {
  }

  void onEnter(Context &ctx) override
  {
    ActionState<Context>::onEnter(ctx);

    CompassReading currentReading = p::compass::device.read();
    Angle current = currentReading.heading;
    Angle finalTarget = target.isAbsolute()
                            ? target
                            : (current + target);

    p::car::compassDriver.reset();
    p::car::compassDriver.setTargetHeading(finalTarget);
  }

  void tick(Context &ctx) override
  {
    p::car::compassDriver.tick();
    if (p::car::compassDriver.hasReachedTarget())
      this->finish(ctx);
  }

  void onExit(Context &ctx) override
  {
    p::car::compassDriver.reset();
  }
};

// Waits for a specified duration in milliseconds before returning to the previous state.
//
// Requirements for Context: none
//
template <typename Context>
struct DelayState : ActionState<Context>
{
  unsigned long durationMs;
  unsigned long startTime = 0;

  DelayState(unsigned long ms)
      : ActionState<Context>("Delay"), durationMs(ms)
  {
  }

  void onEnter(Context &) override
  {
    startTime = millis();
  }

  void tick(Context &ctx) override
  {
    if (millis() - startTime >= durationMs)
      this->finish(ctx);
  }
};

// ===== File: ./include/control/state/command_state.h =====
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
    float speedPercent = cmd.getParamFloat("speed", 50.0f);
    float speed = constrain(speedPercent / 100, 0.2f, 1.0f);

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

// ===== File: ./include/control/state/joystick_control_state.h =====

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
// ===== File: ./include/control/state/state_machine.h =====
#pragma once
#include <Arduino.h>
#include "communication/serial_proxy.h"

struct Event
{
  virtual ~Event() {}
  virtual const char *name() const = 0;
};

#define DEFINE_EVENT_TYPE(NameLiteral)                   \
  static constexpr const char *StaticName = NameLiteral; \
  const char *name() const override { return StaticName; }

template <typename Context>
struct IState
{
  const char *name;
  IState(const char *stateName) : name(stateName) {}
  virtual ~IState() {}

  virtual void onEnter(Context &ctx) {};
  virtual void onResume(Context &ctx) {}
  virtual void onExit(Context &ctx) {};
  virtual bool onEvent(Context &, const Event &) { return false; }
  virtual void tick(Context &ctx) {};
};

template <typename Context, uint8_t MaxStates = 8>
class StateMachine
{
public:
  StateMachine(Context &ctx, IState<Context> &initial)
      : ctx_(ctx), stackSize_(0)
  {
    ctx_.stateMachine = this;
    pushState(initial);
  }

  void tick()
  {
    if (stackSize_ > 0)
      stack_[stackSize_ - 1]->tick(ctx_);
  }

  void pushState(IState<Context> &next)
  {
    if (stackSize_ >= MaxStates)
    {
      SerialProxy::instance().println(F("[StateMachine] ERROR: Stack overflow!"));
      return;
    }

    if (stackSize_ > 0)
    {
      SerialProxy::instance().println("[" + String(stack_[stackSize_ - 1]->name) +
                                      "]: Pushing -> " + next.name);
      stack_[stackSize_ - 1]->onExit(ctx_);
    }

    stack_[stackSize_++] = &next;

    SerialProxy::instance().println("[" + String(next.name) + "]: Enter");
    next.onEnter(ctx_);
  }

  void popState()
  {
    if (stackSize_ == 0)
      return;

    IState<Context> *curr = stack_[stackSize_ - 1];

    SerialProxy::instance().println("[" + String(curr->name) + "]: Pop (Exit)");
    curr->onExit(ctx_);

    stackSize_--;

    if (stackSize_ > 0)
    {
      SerialProxy::instance().println("[" + String(stack_[stackSize_ - 1]->name) + "]: Resume");
      stack_[stackSize_ - 1]->onResume(ctx_);
    }
  }

  void replaceState(IState<Context> &next)
  {
    if (stackSize_ == 0)
    {
      pushState(next);
      return;
    }

    IState<Context> *curr = stack_[stackSize_ - 1];

    SerialProxy::instance().println("[" + String(curr->name) +
                                    "]: Replace -> " + next.name);

    // Exit old state
    curr->onExit(ctx_);

    // Replace without changing stack size
    stack_[stackSize_ - 1] = &next;

    // Enter new state
    SerialProxy::instance().println("[" + String(next.name) + "]: Enter");
    next.onEnter(ctx_);
  }

  void handleEvent(const Event &ev)
  {
    if (stackSize_ == 0)
      return;

    // Traverse top to bottom
    for (int8_t i = stackSize_ - 1; i >= 0; --i)
    {
      IState<Context> *state = stack_[i];

      SerialProxy::instance().println("[" + String(state->name) +
                                      "]: Event '" + ev.name() + "'");

      if (state->onEvent(ctx_, ev))
        break; // event handled -> stop bubbling
    }
  }

  IState<Context> *current() const
  {
    return (stackSize_ == 0) ? nullptr : stack_[stackSize_ - 1];
  }

private:
  Context &ctx_;
  IState<Context> *stack_[MaxStates];
  uint8_t stackSize_;
};
// ===== File: ./include/exercises.h =====
#pragma once
#include <Arduino.h>

namespace ex_w1
{
  void setup();
  void loop();
}

namespace ex_w2_1_2
{
  void setup();
  void loop();
}

namespace ex_w2_1_2_alt
{
  void setup();
  void loop();
}

namespace ex_w3_3_4
{
  void setup();
  void loop();
}

namespace ex_w3_5_6
{
  void setup();
  void loop();
}

namespace ex_w4_4
{
  void setup();
  void loop();
}

namespace ex_w4_5
{
  void setup();
  void loop();
}

namespace ex_5_1
{
  void setup();
  void loop();
}

namespace ex_5_2
{
  void setup();
  void loop();
}

namespace ex_5_5
{
  void setup();
  void loop();
}

namespace ex_5_6
{
  void setup();
  void loop();
}

namespace ex_5_6_alt
{
  void setup();
  void loop();
}

namespace ex_6_4
{
  void setup();
  void loop();
}

namespace ex_7_3
{
  void setup();
  void loop();
}

namespace final
{
  void setup();
  void loop();
}

namespace sandbox
{
  void setup();
  void loop();
}
// ===== File: ./include/hardware/car.h =====
#pragma once
#include <Arduino.h>
#include "hardware/motor.h"
#include "hardware/joystick.h"
#include "control/motor_driver.h"

class Car
{
public:
  Car(Joystick &joystick, Motor &leftMotor, Motor &rightMotor, IMotorDriver &motorDriver);

  void begin();
  void tick();

private:
  bool initialized_ = false;

  Joystick &joystick_;

  Motor &leftMotor_;
  Motor &rightMotor_;
  IMotorDriver &motorDriver_;
};
// ===== File: ./include/hardware/compass.h =====
#pragma once
#include <Arduino.h>
#include <algorithm>
#include "util/directional.h"
#include "pin.h"
#include "communication/i2c.h"

struct CompassReading
{
public:
  Angle heading;
  Direction headingDirection;
};

class Compass
{
public:
  enum class Reg : uint8_t
  {
    HeadingHigh = 0x02,
    HeadingLow = 0x03,
  };

  Compass(uint8_t i2cAddress);

  Compass &begin();

  CompassReading read();
  void setNorthToHeading();

private:
  bool initialized_ = false;

  uint8_t i2cAddress_ = 0x60;

  Angle headingOffset_ = Angle(0);

  Angle readHeading();
};

// ===== File: ./include/hardware/encoder.h =====
#pragma once
#include <Arduino.h>
#include "pin.h"
#include "control/interrupt_dispatcher.h"
#include "control/callback.h"

struct EncoderReading
{
  unsigned long ticks;
  float distanceCm;
};

class Encoder
{
public:
  static constexpr uint8_t MAX_CALLBACKS = 4;

  Encoder(PinBuilder &pinBuilder, float encodingsPerCm);

  Encoder &begin();
  EncoderReading read() const;

  void reset();
  unsigned long getTicks() const;
  float getDistanceCm() const;

  bool addTickCallback(const Callback &cb);

private:
  bool initialized_ = false;
  Pin &pin_;
  float encodingsPerCm_;

  volatile unsigned long tickCount_ = 0;

  Callback callbacks_[MAX_CALLBACKS];
  uint8_t callbackCount_ = 0;

  void onEdge();
};

class EncoderBuilder
{
public:
  EncoderBuilder(PinBuilder &pinBuilder);

  EncoderBuilder &withEncodingsPerCm(float encodingsPerCm);
  Encoder &build();

private:
  PinBuilder &pinBuilder_;
  float encodingsPerCm_ = 27.2305f;
};
// ===== File: ./include/hardware.h =====
#pragma once

#include <hardware/pin.h>
#include <hardware/joystick.h>
#include <hardware/lcd_display.h>
#include <hardware/motor.h>
#include <hardware/compass.h>

#include <hardware/car.h>
// ===== File: ./include/hardware/joystick.h =====
#pragma once
#include <Arduino.h>
#include "pin.h"
#include "control/interrupt_dispatcher.h"

struct JoystickReading
{
  float xUnits; // -1.0 to 1.0
  float yUnits; // -1.0 to 1.0
  bool isIdle;
};

class Joystick
{
public:
  enum class Rotation
  {
    None,
    CW90,
    CW180,
    CW270
  };

  Joystick(PinBuilder &xPinBuilder, PinBuilder &yPinBuilder, PinBuilder &buttonPinBuilder);

  Joystick &begin();

  JoystickReading read();
  bool isButtonPressed();

  void onPress(Callback cb);

private:
  friend class JoystickBuilder;

  bool initialized_ = false;

  Pin &xPin_;
  Pin &yPin_;
  Pin &buttonPin_;

  int maxX_ = 1023;
  int maxY_ = 1023;

  int minX_ = 0;
  int minY_ = 0;

  Rotation rotation_ = Rotation::None;
  int centerX_ = 512;
  int centerY_ = 512;
  float deadzoneUnits_ = 0.00; // 0%
  float curvePotential_ = 1.0; // 1.0 = linear

  // Cached values
  float lastX_ = 0;
  float lastY_ = 0;
  bool lastIdle_ = true;

  void applyRotation(float &x, float &y);
  void clampToUnit(float &x, float &y);
  bool applyDeadzone(float &x, float &y);
  void applyCurve(float &x, float &y);
  float normalizeAxis(int raw, int center, int min, int max);
};

class JoystickBuilder
{
public:
  JoystickBuilder(PinBuilder &xPin, PinBuilder &yPin, PinBuilder &buttonPin);

  JoystickBuilder &setPressDebounce(int debounceDelayMs);
  JoystickBuilder &onPress(Callback cb);

  JoystickBuilder &withRotation(Joystick::Rotation rotation);
  JoystickBuilder &withCenter(int centerX, int centerY);
  JoystickBuilder &withMax(int maxX, int maxY);
  JoystickBuilder &withMin(int minX, int minY);
  JoystickBuilder &withDeadzone(float deadzoneUnits);
  JoystickBuilder &withCurve(float curvePotential);

  Joystick &build();

private:
  PinBuilder &xPinBuilder_;
  PinBuilder &yPinBuilder_;
  PinBuilder &buttonPinBuilder_;

  int debounceDelayMs_ = 0;

  Joystick::Rotation rotation_ = Joystick::Rotation::None;
  bool hasRotation_ = false;

  int maxX_;
  int maxY_;
  int hasMax_ = false;

  int minX_;
  int minY_;
  int hasMin_ = false;

  int centerX_;
  int centerY_;
  bool hasCenter_ = false;

  float deadzoneUnits_;
  bool hasDeadzone_ = false;

  float curvePotential_;
  bool hasCurve_ = false;

  std::vector<Callback> pressHandlers_;
};

// ===== File: ./include/hardware/lcd_display.h =====
#pragma once
#include <Arduino.h>
#include <LiquidCrystal.h>
#include "hardware/pin.h"

class LCDDisplay : public LiquidCrystal
{
public:
  LCDDisplay(Pin &registerSelect, Pin &enable,
             Pin &data4, Pin &data5, Pin &data6, Pin &data7,
             uint8_t cols, uint8_t rows);

  void begin();

  // custom extensions

  void printLine(uint8_t row, const String &text, bool clearRest = true);
  void updateRegion(uint8_t row, uint8_t col, const String &text);

  void centerText(uint8_t row, const String &text);
  void rightAlignText(uint8_t row, const String &text);
  void leftAlignText(uint8_t row, const String &text);

  void clearLine(uint8_t row);
  void printValue(const char *label, int value, uint8_t row);
  void printProgress(uint8_t row, int percent);

private:
  bool initialized_ = false;

  uint8_t cols_;
  uint8_t rows_;
  String lineBuffer_[4]; // store last contents per row
};

class LCDDisplayBuilder
{
public:
  LCDDisplayBuilder &withRegisterSelect(PinBuilder pin);
  LCDDisplayBuilder &withEnable(PinBuilder pin);

  LCDDisplayBuilder &withData4(PinBuilder pin);
  LCDDisplayBuilder &withData5(PinBuilder pin);
  LCDDisplayBuilder &withData6(PinBuilder pin);
  LCDDisplayBuilder &withData7(PinBuilder pin);

  LCDDisplayBuilder &size(uint8_t cols, uint8_t rows);

  LCDDisplay &build();

private:
  Pin *registerSelect_ = nullptr;
  Pin *enable_ = nullptr;
  Pin *d4_ = nullptr;
  Pin *d5_ = nullptr;
  Pin *d6_ = nullptr;
  Pin *d7_ = nullptr;
  uint8_t cols_ = 16;
  uint8_t rows_ = 2;
};

// ===== File: ./include/hardware/motor.h =====
#pragma once
#include <Arduino.h>
#include "pin.h"

class Motor
{
public:
  enum Direction
  {
    BACKWARD = 0,
    FORWARD = 1
  };

  Motor(PinBuilder &powerPin, PinBuilder &directionPin);

  Motor &begin();
  Motor &tick();

  void setPower(float powerUnits); // -1.0 to 1.0
  void stop();

private:
  bool initialized_ = false;

  Pin &powerPin_;
  Pin &directionPin_;
  bool motorStateChanged_ = true;

  const unsigned int maxPower_ = 255;

  float currentPowerUnits_ = 0; // 0 to 1.0
  Direction currentDirection_ = FORWARD;
};
// ===== File: ./include/hardware/pin.h =====
#pragma once
#include <Arduino.h>
#include "control/interrupt_dispatcher.h"

class Pin
{
public:
  explicit Pin(uint8_t id);

  Pin &begin();

  uint8_t id();
  InterruptDispatcher &interruptDispatcher();

  void setHigh();
  void setLow();
  void setToggle();

  bool isHigh();
  bool isLow();
  int read();

  int analogReadValue();
  void analogWriteValue(int value);

private:
  bool initialized_ = false;

  uint8_t id_;
  uint8_t mode_ = INPUT;
  uint8_t initialValue_ = LOW;

  InterruptDispatcher interruptDispatcher_;

  friend class PinBuilder;
};

class PinBuilder
{
public:
  explicit PinBuilder(uint8_t id);

  Pin &build();

  PinBuilder &asInput();
  PinBuilder &asInputPullup();
  PinBuilder &asOutput();

  PinBuilder &withInitialValue(uint8_t value);

  PinBuilder &withFallingInterrupt(Callback cb);
  PinBuilder &withRisingInterrupt(Callback cb);
  PinBuilder &withChangeInterrupt(Callback cb);
  PinBuilder &withLowInterrupt(Callback cb);
  PinBuilder &withHighInterrupt(Callback cb);

  PinBuilder &withDebounce(unsigned long ms);

private:
  Pin pin_;
};

// ===== File: ./include/physical.h =====
#pragma once
#include <Arduino.h>
#include "hardware.h"

namespace physical
{

  namespace lcd
  {
    inline PinBuilder rs(51);
    inline PinBuilder en(50);
    inline PinBuilder d4(48);
    inline PinBuilder d5(49);
    inline PinBuilder d6(47);
    inline PinBuilder d7(46);

    inline LCDDisplay &device = LCDDisplayBuilder()
                                    .withRegisterSelect(rs)
                                    .withEnable(en)
                                    .withData4(d4)
                                    .withData5(d5)
                                    .withData6(d6)
                                    .withData7(d7)
                                    .size(20, 4)
                                    .build();
  }

  namespace joystick
  {
    inline PinBuilder x(A8);
    inline PinBuilder y(A7);
    inline PinBuilder btn(19);

    inline Joystick &device = JoystickBuilder(x, y, btn)
                                  .withRotation(Joystick::Rotation::None)
                                  .withCenter(500, 493)
                                  .withMin(19, 19)
                                  .withMax(1007, 1007)
                                  // .withDeadzone(0.05)
                                  .setPressDebounce(200)
                                  .build();
  }

  namespace compass
  {
    inline uint8_t address = 0x60;
    inline Compass device = Compass(address);
  }

  namespace motor
  {
    inline PinBuilder leftPwm(9);
    inline PinBuilder leftDirection(7);

    inline PinBuilder rightPwm(10);
    inline PinBuilder rightDirection(8);

    inline PinBuilder encoderLeft(2);
    inline PinBuilder encoderRight(3);

    inline auto &leftEncoder = EncoderBuilder(encoderLeft)
                                   .withEncodingsPerCm(27.2305f)
                                   .build();

    inline auto &rightEncoder = EncoderBuilder(encoderRight)
                                    .withEncodingsPerCm(27.2305f)
                                    .build();

    inline Motor leftMotor(leftPwm, leftDirection);
    inline Motor rightMotor(rightPwm, rightDirection);
  }

  namespace car
  {
    inline CompassHeadingMotorDriver compassDriver{
        motor::leftMotor,
        motor::rightMotor,
        compass::device};
    inline JoystickMotorDriver joystickDriver{
        motor::leftMotor,
        motor::rightMotor,
        joystick::device};
    inline DistanceMotorDriver distanceDriver{
        motor::leftMotor,
        motor::rightMotor,
        motor::leftEncoder};

    inline Car device(joystick::device, motor::leftMotor, motor::rightMotor, joystickDriver);

    inline void beginAll()
    {
      lcd::device.begin();

      joystick::device.begin();

      compass::device.begin();

      motor::leftEncoder.begin();
      motor::rightEncoder.begin();

      motor::leftMotor.begin();
      motor::rightEncoder.begin();

      joystickDriver.begin();
      compassDriver.begin();
      distanceDriver.begin();
    }
  }
}
// ===== File: ./include/sandbox.h =====
#pragma once
#include <Arduino.h>
#include "physical.h"
#include "control/state/state_machine.h"

namespace p = physical;

class Sandbox
{

public:
  void setup()
  {
    Serial.begin(9600);

    p::lcd::device.begin();
    p::car::device.begin();
  }

  void loop()
  {

    JoystickReading reading = p::joystick::device.read();
    p::lcd::device.printLine(0, "X: " + String(reading.xUnits) + " Y: " + String(reading.yUnits));
    p::car::device.tick();

    delay(100);
  }
};

namespace sandbox
{
  inline Sandbox instance;

  inline void setup() { instance.setup(); }
  inline void loop() { instance.loop(); }
}
// ===== File: ./include/util/directional.h =====
#pragma once
#include <Arduino.h>
#include <math.h>

struct Angle
{
public:
  explicit Angle(float deg = 0)
      : degrees_(wrap(deg)), isAbsolute_(true) {}

  static Angle absolute(float deg) { return Angle(deg, true); }
  static Angle relative(float deg) { return Angle(deg, false); }

  float value() const { return degrees_; }
  bool isAbsolute() const { return isAbsolute_; }

  Angle differenceTo(const Angle &other) const
  {
    float a = wrap(degrees_);
    float b = wrap(other.degrees_);

    float d = b - a;

    if (d > 180)
      d -= 360;
    if (d < -180)
      d += 360;

    return Angle::relative(d);
  }

  Angle operator+(float deg) const { return Angle(degrees_ + deg, isAbsolute_); }
  Angle operator-(float deg) const { return *this + (-deg); }

  Angle operator+(const Angle &other) const { return Angle(degrees_ + other.degrees_, isAbsolute_ || other.isAbsolute_); }
  Angle operator-(const Angle &other) const { return *this + Angle(-other.degrees_, other.isAbsolute_); }

  Angle operator*(const float factor) const { return Angle(degrees_ * factor, isAbsolute_); }

private:
  float degrees_;
  bool isAbsolute_;

  Angle(float deg, bool isAbs)
      : degrees_(isAbs ? wrap(deg) : deg), isAbsolute_(isAbs) {}

  static float wrap(float d)
  {
    d = fmodf(d, 360.0f);
    return (d < 0 ? d + 360.0f : d);
  }
};

struct Direction
{
public:
  enum class Compass
  {
    North,
    NorthEast,
    East,
    SouthEast,
    South,
    SouthWest,
    West,
    NorthWest,
  };

  explicit Direction(Compass v = Compass::North) : value(v) {}

  static Direction fromAngle(const Angle &angle)
  {
    float deg = angle.value();
    int index = static_cast<int>(floor((deg + 22.5f) / 45.0f)) % 8;

    return Direction(directions_[index]);
  }

  const Angle toAngle()
  {
    return Angle(static_cast<int>(value) * 45.0f);
  }

  const char *toString() const
  {
    switch (value)
    {
    case Compass::North:
      return "N";
    case Compass::NorthEast:
      return "NE";
    case Compass::East:
      return "E";
    case Compass::SouthEast:
      return "SE";
    case Compass::South:
      return "S";
    case Compass::SouthWest:
      return "SW";
    case Compass::West:
      return "W";
    case Compass::NorthWest:
      return "NW";
    default:
      return "?";
    }
  }

  bool operator==(const Direction &other) const { return value == other.value; }
  bool operator!=(const Direction &other) const { return !(*this == other); }

private:
  Compass value;

  // ordered in clockwise direction, beginning from north
  static constexpr Compass directions_[8] = {
      Compass::North, Compass::NorthEast, Compass::East, Compass::SouthEast,
      Compass::South, Compass::SouthWest, Compass::West, Compass::NorthWest};
};


// === Source Files ===

// ===== File: ./src/communication/i2c.cpp =====
#include <Arduino.h>
#include "communication/i2c.h"
#include <Wire.h>

I2CBus &I2CBus::begin()
{
  if (initialized_)
    return *this;

  initialized_ = true;

  Wire.begin();

  return *this;
}

byte I2CBus::readByte(uint8_t address, uint8_t reg)
{
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.endTransmission(false);

  Wire.requestFrom(address, 1, (int)true);

  if (Wire.available())
  {
    return Wire.read();
  }
  return 0;
}
// ===== File: ./src/control/compass_heading_motor_driver.cpp =====
#include "control/motor_driver.h"

CompassHeadingMotorDriver::CompassHeadingMotorDriver(
    Motor &left, Motor &right, Compass &compass)
    : leftMotor_(left), rightMotor_(right), compass_(compass)
{
}

void CompassHeadingMotorDriver::begin()
{
  if (initialized_)
    return;

  leftMotor_.begin();
  rightMotor_.begin();
  compass_.begin();

  initialized_ = true;
}

void CompassHeadingMotorDriver::reset()
{
  targetSet_ = false;
  reachedTarget_ = false;
  stopMotors();
}

void CompassHeadingMotorDriver::setTargetHeading(const Angle &heading)
{
  targetHeading_ = heading;
  targetSet_ = true;
  reachedTarget_ = false;
}

void CompassHeadingMotorDriver::setTolerance(const Angle &tolerance)
{
  tolerance_ = tolerance;
}

bool CompassHeadingMotorDriver::hasReachedTarget() const
{
  // reached when turning is complete AND no active target is set
  return reachedTarget_ && !targetSet_;
}

void CompassHeadingMotorDriver::tick()
{
  if (!initialized_)
    return;

  // stop motors if no target is set
  if (!targetSet_)
  {
    stopMotors();
    return;
  }

  // perform turning towards target
  if (approachTarget())
  {
    reachedTarget_ = true;
    targetSet_ = false;

    stopMotors();
  }

  leftMotor_.tick();
  rightMotor_.tick();
}

bool CompassHeadingMotorDriver::approachTarget()
{
  CompassReading reading = compass_.read();
  Angle currentHeading = reading.heading;

  Angle diff = targetHeading_.differenceTo(currentHeading);
  float headingError = diff.value();
  float absErr = abs(headingError);

  // check if within tolerance
  if (absErr <= tolerance_.value())
  {
    return true;
  }

  // small error -> small power
  // large error -> quadratically larger power
  float wantedPower = (absErr * absErr) / 2000.0f;
  float power = constrain(wantedPower, 0.2f, 1.0f);

  if (headingError > 0)
  {
    // turn left
    leftMotor_.setPower(-power);
    rightMotor_.setPower(power);
  }
  else
  {
    // turn right
    leftMotor_.setPower(power);
    rightMotor_.setPower(-power);
  }

  return false; // not finished turning
}

void CompassHeadingMotorDriver::stopMotors()
{
  leftMotor_.stop();
  rightMotor_.stop();

  leftMotor_.tick();
  rightMotor_.tick();
}

// ===== File: ./src/control/distance_motor_driver.cpp =====
#include "control/motor_driver.h"

DistanceMotorDriver::DistanceMotorDriver(Motor &leftMotor, Motor &rightMotor, Encoder &encoder)
    : leftMotor_(leftMotor),
      rightMotor_(rightMotor),
      encoder_(encoder)
{
}

void DistanceMotorDriver::begin()
{
  if (initialized_)
    return;
  initialized_ = true;

  stopMotors();
  encoder_.reset();

  targetSet_ = false;
  reachedTarget_ = false;

  encoder_.addTickCallback(makeCallback(this, &DistanceMotorDriver::onEncoderTick));
}

void DistanceMotorDriver::setTarget(float distanceCm, float power)
{
  if (!initialized_)
    return;

  targetDistanceCm_ = distanceCm;
  drivePower_ = constrain(power, -1.0f, 1.0f);

  startDistanceCm_ = encoder_.getDistanceCm();

  targetSet_ = true;
  reachedTarget_ = false;
}

void DistanceMotorDriver::reset()
{
  targetSet_ = false;
  reachedTarget_ = false;
  encoder_.reset();
}

bool DistanceMotorDriver::hasReachedTarget() const
{
  return targetSet_ && reachedTarget_;
}

void DistanceMotorDriver::tick()
{
  if (!initialized_ || !targetSet_ || reachedTarget_)
  {

    return;
  }

  driveStraight();
}

void DistanceMotorDriver::onEncoderTick()
{
  if (!targetSet_ || reachedTarget_)
    return;

  float distanceSinceStartCm = encoder_.getDistanceCm() - startDistanceCm_;
  if (distanceSinceStartCm >= targetDistanceCm_)
  {
    stopMotors();
    reachedTarget_ = true;
  }
}

void DistanceMotorDriver::driveStraight()
{
  leftMotor_.setPower(drivePower_);
  rightMotor_.setPower(drivePower_);

  leftMotor_.tick();
  rightMotor_.tick();
}

void DistanceMotorDriver::stopMotors()
{
  leftMotor_.stop();
  rightMotor_.stop();

  leftMotor_.tick();
  rightMotor_.tick();
}

// ===== File: ./src/control/interrupt_dispatcher.cpp =====
#include <Arduino.h>
#include "control/interrupt_dispatcher.h"

// Static dispatcher registry
InterruptDispatcher *InterruptDispatcher::dispatchers_[NUM_DIGITAL_PINS] = {nullptr};

// Generate trampoline table for each possible digital pin
using ISRHandler = void (*)();
ISRHandler trampolines[NUM_DIGITAL_PINS] = {
    Trampoline<0>::handler, Trampoline<1>::handler, Trampoline<2>::handler,
    Trampoline<3>::handler, Trampoline<4>::handler, Trampoline<5>::handler,
    Trampoline<6>::handler, Trampoline<7>::handler, Trampoline<8>::handler,
    Trampoline<9>::handler, Trampoline<10>::handler, Trampoline<11>::handler,
    Trampoline<12>::handler, Trampoline<13>::handler, Trampoline<14>::handler,
    Trampoline<15>::handler, Trampoline<16>::handler, Trampoline<17>::handler,
    Trampoline<18>::handler, Trampoline<19>::handler, Trampoline<20>::handler,
    Trampoline<21>::handler, Trampoline<22>::handler, Trampoline<23>::handler,
    Trampoline<24>::handler, Trampoline<25>::handler, Trampoline<26>::handler,
    Trampoline<27>::handler, Trampoline<28>::handler, Trampoline<29>::handler,
    Trampoline<30>::handler, Trampoline<31>::handler, Trampoline<32>::handler,
    Trampoline<33>::handler, Trampoline<34>::handler, Trampoline<35>::handler,
    Trampoline<36>::handler, Trampoline<37>::handler, Trampoline<38>::handler,
    Trampoline<39>::handler, Trampoline<40>::handler, Trampoline<41>::handler,
    Trampoline<42>::handler, Trampoline<43>::handler, Trampoline<44>::handler,
    Trampoline<45>::handler, Trampoline<46>::handler, Trampoline<47>::handler,
    Trampoline<48>::handler, Trampoline<49>::handler, Trampoline<50>::handler,
    Trampoline<51>::handler, Trampoline<52>::handler, Trampoline<53>::handler};

InterruptDispatcher::InterruptDispatcher(uint8_t pinId, int lastState)
    : pinId_(pinId), lastState_(lastState)
{
  dispatchers_[pinId] = this;
}

void InterruptDispatcher::begin()
{
  if (initialized_)
    return;

  if (!hasHandlers())
    return;

  int irq = digitalPinToInterrupt(pinId_);
  if (irq == NOT_AN_INTERRUPT)
  {
    Serial.print("ERROR: Pin ");
    Serial.print(pinId_);
    Serial.println(" does not support interrupts!");
    return;
  }

  // Always attach in CHANGE. Software decides edges/levels.
  attachInterrupt(irq, trampolines[pinId_], CHANGE);

  lastState_ = fastRead(pinId_);
  lastInterruptTime_ = millis();
  initialized_ = true;
}

void InterruptDispatcher::setDebounce(unsigned long ms)
{
  debounceMs_ = ms;
}

bool InterruptDispatcher::hasHandlers() const
{
  return !lowHandlers_.empty() ||
         !highHandlers_.empty() ||
         !changeHandlers_.empty() ||
         !risingHandlers_.empty() ||
         !fallingHandlers_.empty();
}

inline int InterruptDispatcher::fastRead(uint8_t pin)
{
#if defined(ARDUINO_ARCH_AVR)
  uint8_t port = digitalPinToPort(pin);
  if (port == NOT_A_PIN)
    return digitalRead(pin);
  volatile uint8_t *reg = portInputRegister(port);
  uint8_t mask = digitalPinToBitMask(pin);
  return ((*reg & mask) ? HIGH : LOW);
#else
  return digitalRead(pin);
#endif
}

void InterruptDispatcher::handleInterrupt()
{
  unsigned long now = millis();

  if (debounceMs_ && (now - lastInterruptTime_ < debounceMs_))
    return;

  int oldState = lastState_;
  int newState = fastRead(pinId_);

  // Latch time/state before calling user code
  lastInterruptTime_ = now;
  lastState_ = newState;

  // Dispatch
  for (auto &cb : changeHandlers_)
    cb();

  if (oldState == LOW && newState == HIGH)
  {
    for (auto &cb : risingHandlers_)
      cb();
  }
  else if (oldState == HIGH && newState == LOW)
  {
    for (auto &cb : fallingHandlers_)
      cb();
  }

  if (newState == HIGH)
  {
    for (auto &cb : highHandlers_)
      cb();
  }
  else
  {
    for (auto &cb : lowHandlers_)
      cb();
  }
}

void InterruptDispatcher::onLow(Callback cb) { lowHandlers_.push_back(cb); }
void InterruptDispatcher::onHigh(Callback cb) { highHandlers_.push_back(cb); }
void InterruptDispatcher::onChange(Callback cb) { changeHandlers_.push_back(cb); }
void InterruptDispatcher::onRising(Callback cb) { risingHandlers_.push_back(cb); }
void InterruptDispatcher::onFalling(Callback cb) { fallingHandlers_.push_back(cb); }

// ===== File: ./src/control/joystick_motor_driver.cpp =====
#include <control/motor_driver.h>

JoystickMotorDriver::JoystickMotorDriver(Motor &leftMotor, Motor &rightMotor, Joystick &joystick)
    : leftMotor_(leftMotor), rightMotor_(rightMotor), joystick_(joystick) {}

void JoystickMotorDriver::begin()
{
  if (initialized_)
    return;

  leftMotor_.begin();
  rightMotor_.begin();
  joystick_.begin();

  initialized_ = true;
}

void JoystickMotorDriver::tick()
{
  JoystickReading joystickState = joystick_.read();

  float leftPowerUnits = joystickState.yUnits + joystickState.xUnits;
  leftPowerUnits = constrain(leftPowerUnits, -1.0, 1.0);
  leftMotor_.setPower(leftPowerUnits);

  float rightPowerUnits = joystickState.yUnits - joystickState.xUnits;
  rightPowerUnits = constrain(rightPowerUnits, -1.0, 1.0);
  rightMotor_.setPower(rightPowerUnits);

  // Serial.println("Left Power: " + String(leftPowerUnits) + ", Right Power: " + String(rightPowerUnits));
}

JoystickVectorMotorDriver::JoystickVectorMotorDriver(Motor &leftMotor, Motor &rightMotor, Joystick &joystick)
    : leftMotor_(leftMotor), rightMotor_(rightMotor), joystick_(joystick) {}

void JoystickVectorMotorDriver::begin()
{
  if (initialized_)
    return;

  leftMotor_.begin();
  rightMotor_.begin();
  joystick_.begin();

  initialized_ = true;
}

void JoystickVectorMotorDriver::tick()
{
  // TODO: Implement relative driving logic. Compass needed.
}
// ===== File: ./src/exercises/final.cpp =====
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
// ===== File: ./src/hardware/car.cpp =====
#include <hardware/car.h>

Car::Car(Joystick &joystick, Motor &leftMotor, Motor &rightMotor, IMotorDriver &motorDriver)
    : joystick_(joystick), leftMotor_(leftMotor), rightMotor_(rightMotor), motorDriver_(motorDriver) {}

void Car::begin()
{
  if (initialized_)
    return;

  joystick_.begin();
  leftMotor_.begin();
  rightMotor_.begin();
  motorDriver_.begin();

  initialized_ = true;
}

void Car::tick()
{
  // update motor state
  motorDriver_.tick();

  // apply motor state
  leftMotor_.tick();
  rightMotor_.tick();
}
// ===== File: ./src/hardware/compass.cpp =====
#include <Arduino.h>
#include "hardware/compass.h"
#include "communication/i2c.h"

Compass::Compass(uint8_t address) : i2cAddress_(address) {}

Compass &Compass::begin()
{
  if (initialized_)
  {
    return *this;
  }

  I2CBus::instance().begin();

  return *this;
}

CompassReading Compass::read()
{
  Angle heading = readHeading();

  return CompassReading{
      heading,
      Direction::fromAngle(heading)};
}

void Compass::setNorthToHeading()
{
  Angle currentHeading = readHeading();
  headingOffset_ = currentHeading * -1;
}

Angle Compass::readHeading()
{
  byte high_byte = I2CBus::instance().readByte(i2cAddress_, 2);
  byte low_byte = I2CBus::instance().readByte(i2cAddress_, 3);

  int heading = (high_byte << 8) | low_byte; // Combine high and low bytes

  float headingDeg = heading / 10.0; // Convert to degrees (range is 0-3599 -> 0.0-359.9 degrees)

  return Angle(headingDeg) + headingOffset_;
}
// ===== File: ./src/hardware/encoder.cpp =====
#include "hardware/encoder.h"

Encoder::Encoder(PinBuilder &pinBuilder, float encodingsPerCm)
    : pin_(pinBuilder
               .asInputPullup()
               .withChangeInterrupt(makeCallback(this, &Encoder::onEdge))
               .build()),
      encodingsPerCm_(encodingsPerCm)
{
}

Encoder &Encoder::begin()
{
  if (initialized_)
    return *this;
  initialized_ = true;

  pin_.begin();
  reset();

  return *this;
}

void Encoder::reset()
{
  tickCount_ = 0;
}

EncoderReading Encoder::read() const
{
  EncoderReading r;
  r.ticks = tickCount_;
  r.distanceCm = r.ticks / encodingsPerCm_;
  return r;
}

unsigned long Encoder::getTicks() const
{
  unsigned long t = tickCount_;
  return t;
}

float Encoder::getDistanceCm() const
{
  unsigned long t = tickCount_;
  return t / encodingsPerCm_;
}

bool Encoder::addTickCallback(const Callback &cb)
{
  if (callbackCount_ >= MAX_CALLBACKS)
    return false;

  callbacks_[callbackCount_++] = cb;
  return true;
}

void Encoder::onEdge() // ISR
{
  tickCount_++;

  // dispatch callbacks, must run quickly!
  for (uint8_t i = 0; i < callbackCount_; i++)
    callbacks_[i]();
}

// ----------------- EncoderBuilder ----------------

EncoderBuilder::EncoderBuilder(PinBuilder &pinBuilder)
    : pinBuilder_(pinBuilder)
{
}

EncoderBuilder &EncoderBuilder::withEncodingsPerCm(float encodingsPerCm)
{
  encodingsPerCm_ = encodingsPerCm;
  return *this;
}

Encoder &EncoderBuilder::build()
{
  return *(new Encoder(pinBuilder_, encodingsPerCm_));
}
// ===== File: ./src/hardware/joystick.cpp =====
#include <Arduino.h>
#include "hardware/joystick.h"

Joystick::Joystick(PinBuilder &xPinBuilder, PinBuilder &yPinBuilder, PinBuilder &buttonPinBuilder)
    : xPin_(xPinBuilder.asInput().build()),
      yPin_(yPinBuilder.asInput().build()),
      buttonPin_(buttonPinBuilder.asInputPullup().build())
{
}

Joystick &Joystick::begin()
{
  if (initialized_)
    return *this;

  xPin_.begin();
  yPin_.begin();
  buttonPin_.begin();

  initialized_ = true;
  return *this;
}

JoystickReading Joystick::read()
{
  int rawX = xPin_.analogReadValue();
  int rawY = yPin_.analogReadValue();

  // normalize
  float normX = normalizeAxis(rawX, centerX_, minX_, maxX_);
  float normY = -normalizeAxis(rawY, centerY_, minY_, maxY_); // Negate Y to have up = positive

  // rotate, clamp, correct deadzone, curve shaping
  applyRotation(normX, normY);
  clampToUnit(normX, normY);
  bool isIdle = applyDeadzone(normX, normY);

  if (!isIdle)
    applyCurve(normX, normY);

  // cache values for consistent partial reads
  lastX_ = normX;
  lastY_ = normY;
  lastIdle_ = isIdle;

  return JoystickReading{normX, normY, isIdle};
}

void Joystick::applyRotation(float &x, float &y)
{
  float oldX = x, oldY = y;

  switch (rotation_)
  {
  case Rotation::None:
    // no change
    break;
  case Rotation::CW90:
    x = oldY;
    y = -oldX;
    break;
  case Rotation::CW180:
    x = -oldX;
    y = -oldY;
    break;
  case Rotation::CW270:
    x = -oldY;
    y = oldX;
    break;
  }
}

void Joystick::clampToUnit(float &x, float &y)
{
  x = constrain(x, -1.0f, 1.0f);
  y = constrain(y, -1.0f, 1.0f);
}

bool Joystick::applyDeadzone(float &x, float &y)
{
  float magnitude = sqrt(x * x + y * y);
  if (magnitude < deadzoneUnits_)
  {
    x = 0;
    y = 0;
    return true; // joystick idle
  }
  return false;
}

void Joystick::applyCurve(float &x, float &y)
{
  if (curvePotential_ != 1.0f)
  {
    x = copysign(pow(fabs(x), curvePotential_), x);
    y = copysign(pow(fabs(y), curvePotential_), y);
  }
}

bool Joystick::isButtonPressed()
{
  return buttonPin_.isLow(); // pull-up active, LOW = pressed
}

void Joystick::onPress(Callback cb)
{
  buttonPin_.interruptDispatcher().onChange(cb);
}

float Joystick::normalizeAxis(int raw, int center, int min, int max)
{
  if (raw >= center)
    // Right/up side
    return (raw - center) / float(max - center);

  // Left/down side
  return (raw - center) / float(center - min);
}

// ----------------- JoystickBuilder ----------------

JoystickBuilder::JoystickBuilder(PinBuilder &xPin, PinBuilder &yPin, PinBuilder &buttonPin)
    : xPinBuilder_(xPin), yPinBuilder_(yPin), buttonPinBuilder_(buttonPin) {}

JoystickBuilder &JoystickBuilder::setPressDebounce(int debounceDelayMs)
{
  debounceDelayMs_ = debounceDelayMs;
  return *this;
}

JoystickBuilder &JoystickBuilder::onPress(Callback cb)
{
  pressHandlers_.push_back(cb);
  return *this;
}

JoystickBuilder &JoystickBuilder::withRotation(Joystick::Rotation rotation)
{
  rotation_ = rotation;
  hasRotation_ = true;
  return *this;
}

JoystickBuilder &JoystickBuilder::withCenter(int centerX, int centerY)
{
  centerX_ = centerX;
  centerY_ = centerY;
  hasCenter_ = true;
  return *this;
}

JoystickBuilder &JoystickBuilder::withMax(int maxX, int maxY)
{
  maxX_ = maxX;
  maxY_ = maxY;
  hasMax_ = true;
  return *this;
}

JoystickBuilder &JoystickBuilder::withMin(int minX, int minY)
{
  minX_ = minX;
  minY_ = minY;
  hasMin_ = true;
  return *this;
}

JoystickBuilder &JoystickBuilder::withDeadzone(float deadzoneUnits)
{
  deadzoneUnits_ = deadzoneUnits;
  hasDeadzone_ = true;
  return *this;
}

JoystickBuilder &JoystickBuilder::withCurve(float curvePotential)
{
  curvePotential_ = curvePotential;
  hasDeadzone_ = true;
  return *this;
}

Joystick &JoystickBuilder::build()
{

  Joystick *js = new Joystick(
      xPinBuilder_,
      yPinBuilder_,
      buttonPinBuilder_
          .withDebounce(debounceDelayMs_));

  if (hasRotation_)
    js->rotation_ = rotation_;

  if (hasCenter_)
  {
    js->centerX_ = centerX_;
    js->centerY_ = centerY_;
  }

  if (hasMax_)
  {
    js->maxX_ = maxX_;
    js->maxY_ = maxY_;
  }

  if (hasMin_)
  {
    js->minX_ = minX_;
    js->minY_ = minY_;
  }

  if (hasDeadzone_)
    js->deadzoneUnits_ = deadzoneUnits_;

  if (hasCurve_)
    js->curvePotential_ = curvePotential_;

  // Attach all pre-configured handlers
  for (auto &cb : pressHandlers_)
  {
    js->onPress(cb);
  }

  return *js;
}

// ===== File: ./src/hardware/lcd_display.cpp =====
#include <Arduino.h>
#include "hardware/lcd_display.h"
#include <algorithm>

LCDDisplay::LCDDisplay(Pin &registerSelect, Pin &enable,
                       Pin &data4, Pin &data5, Pin &data6, Pin &data7,
                       uint8_t cols, uint8_t rows)
    : LiquidCrystal(registerSelect.id(), enable.id(),
                    data4.id(), data5.id(), data6.id(), data7.id()),
      cols_(cols), rows_(rows) {}

void LCDDisplay::begin()
{
  if (initialized_)
    return;

  LiquidCrystal::begin(cols_, rows_);
  for (uint8_t r = 0; r < rows_; r++)
  {
    lineBuffer_[r] = "";
  }

  initialized_ = true;
}

void LCDDisplay::clearLine(uint8_t row)
{
  if (row >= rows_)
    return;

  setCursor(0, row);
  for (uint8_t i = 0; i < cols_; i++)
  {
    print(' ');
  }
  setCursor(0, row);
  lineBuffer_[row] = String(cols_, ' ');
}

void LCDDisplay::printLine(uint8_t row, const String &text, bool clearRest)
{
  if (row >= rows_)
    return;

  // Avoid flicker: only update if different
  if (lineBuffer_[row] == text)
    return;

  setCursor(0, row);
  print(text);

  if (clearRest && text.length() < cols_)
  {
    for (uint8_t i = text.length(); i < cols_; i++)
    {
      print(' ');
    }
  }

  lineBuffer_[row] = text;
}

void LCDDisplay::updateRegion(uint8_t row, uint8_t col, const String &text)
{
  if (row >= rows_ || col >= cols_)
    return;

  setCursor(col, row);
  print(text);

  // update buffer only where new text goes
  String &buf = lineBuffer_[row];
  if (buf.length() < cols_)
    buf += String(' ', cols_ - buf.length());

  for (uint8_t i = 0; i < text.length() && (col + i) < cols_; i++)
  {
    buf[col + i] = text[i];
  }
}

void LCDDisplay::centerText(uint8_t row, const String &text)
{
  if (row >= rows_)
    return;
  int startCol = std::max(0, (cols_ - (int)text.length()) / 2);
  printLine(row, String(' ', startCol) + text, true);
}

void LCDDisplay::rightAlignText(uint8_t row, const String &text)
{
  if (row >= rows_)
    return;
  int startCol = std::max(0, (int)(cols_ - text.length()));
  printLine(row, String(' ', startCol) + text, true);
}

void LCDDisplay::leftAlignText(uint8_t row, const String &text)
{
  if (row >= rows_)
    return;
  printLine(row, text, true);
}

void LCDDisplay::printValue(const char *label, int value, uint8_t row)
{
  String text = String(label) + ": " + String(value);
  printLine(row, text);
}

void LCDDisplay::printProgress(uint8_t row, int percent)
{
  if (row >= rows_)
    return;
  int filled = (percent * cols_) / 100;
  String bar;
  for (int i = 0; i < filled; i++)
    bar += (char)255;
  for (int i = filled; i < cols_; i++)
    bar += ' ';
  printLine(row, bar, false);
}

// ----------------- LCDDisplayBuilder ----------------
LCDDisplayBuilder &LCDDisplayBuilder::withRegisterSelect(PinBuilder pin)
{
  registerSelect_ = &pin.build();
  return *this;
}

LCDDisplayBuilder &LCDDisplayBuilder::withEnable(PinBuilder pin)
{
  enable_ = &pin.asOutput().build();
  return *this;
}

LCDDisplayBuilder &LCDDisplayBuilder::withData4(PinBuilder pin)
{
  d4_ = &pin.asOutput().build();
  return *this;
}

LCDDisplayBuilder &LCDDisplayBuilder::withData5(PinBuilder pin)
{
  d5_ = &pin.asOutput().build();
  return *this;
}

LCDDisplayBuilder &LCDDisplayBuilder::withData6(PinBuilder pin)
{
  d6_ = &pin.asOutput().build();
  return *this;
}

LCDDisplayBuilder &LCDDisplayBuilder::withData7(PinBuilder pin)
{
  d7_ = &pin.asOutput().build();
  return *this;
}

LCDDisplayBuilder &LCDDisplayBuilder::size(uint8_t cols, uint8_t rows)
{
  cols_ = cols;
  rows_ = rows;
  return *this;
}

LCDDisplay &LCDDisplayBuilder::build()
{
  static LCDDisplay lcd(*registerSelect_, *enable_, *d4_, *d5_, *d6_, *d7_, cols_, rows_);
  return lcd;
}

// ===== File: ./src/hardware/motor.cpp =====
#include <Arduino.h>
#include "hardware/motor.h"

Motor::Motor(PinBuilder &powerPinBuilder, PinBuilder &directionPinBuilder) : powerPin_(powerPinBuilder.asOutput().build()),
                                                                             directionPin_(directionPinBuilder.asOutput().build())
{
}

Motor &Motor::begin()
{
  if (initialized_)
    return *this;

  powerPin_.begin();
  directionPin_.begin();

  initialized_ = true;
  return *this;
}

Motor &Motor::tick()
{
  if (!motorStateChanged_)
    return *this;

  int powerValue = map(currentPowerUnits_ * 100, 0, 100, 0, maxPower_);
  powerPin_.analogWriteValue(powerValue);

  switch (currentDirection_)
  {
  case Direction::FORWARD:
    directionPin_.setLow();
    break;

  case Direction::BACKWARD:
    directionPin_.setHigh();
    break;
  }

  return *this;
}

void Motor::setPower(float powerUnit)
{
  currentDirection_ = (powerUnit >= 0) ? Direction::FORWARD : Direction::BACKWARD;
  currentPowerUnits_ = abs(powerUnit);
  motorStateChanged_ = true;
}

void Motor::stop()
{
  setPower(0);
}

// ===== File: ./src/hardware/pin.cpp =====
#include "hardware/pin.h"

Pin::Pin(uint8_t pinId)
    : id_(pinId), interruptDispatcher_(pinId) {}

Pin &Pin::begin()
{
  if (initialized_)
    return *this;

  // configure mode
  pinMode(id_, mode_);

  // set initial value for outputs
  if (mode_ == OUTPUT)
  {
    digitalWrite(id_, initialValue_);
  }

  interruptDispatcher_.begin();

  initialized_ = true;
  return *this;
}

uint8_t Pin::id() { return id_; }

InterruptDispatcher &Pin::interruptDispatcher()
{
  return interruptDispatcher_;
}

void Pin::setHigh() { digitalWrite(id_, HIGH); }
void Pin::setLow() { digitalWrite(id_, LOW); }
void Pin::setToggle() { digitalWrite(id_, isHigh() ? LOW : HIGH); }

bool Pin::isHigh() { return read() == HIGH; }
bool Pin::isLow() { return read() == LOW; }
int Pin::read() { return digitalRead(id_); }

int Pin::analogReadValue() { return analogRead(id_); }
void Pin::analogWriteValue(int v) { analogWrite(id_, v); }

// ----------------- PinBuilder ----------------

PinBuilder::PinBuilder(uint8_t id)
    : pin_(id) {}

Pin &PinBuilder::build()
{
  return pin_;
}

PinBuilder &PinBuilder::asInput()
{
  pin_.mode_ = INPUT;
  return *this;
}

PinBuilder &PinBuilder::asInputPullup()
{
  pin_.mode_ = INPUT_PULLUP;
  return *this;
}

PinBuilder &PinBuilder::asOutput()
{
  pin_.mode_ = OUTPUT;
  return *this;
}

PinBuilder &PinBuilder::withInitialValue(uint8_t value)
{
  pin_.initialValue_ = value;
  return *this;
}

PinBuilder &PinBuilder::withFallingInterrupt(Callback cb)
{
  pin_.interruptDispatcher_.onFalling(cb);
  return *this;
}

PinBuilder &PinBuilder::withRisingInterrupt(Callback cb)
{
  pin_.interruptDispatcher_.onRising(cb);
  return *this;
}

PinBuilder &PinBuilder::withChangeInterrupt(Callback cb)
{
  pin_.interruptDispatcher_.onChange(cb);
  return *this;
}

PinBuilder &PinBuilder::withLowInterrupt(Callback cb)
{
  pin_.interruptDispatcher_.onLow(cb);
  return *this;
}

PinBuilder &PinBuilder::withHighInterrupt(Callback cb)
{
  pin_.interruptDispatcher_.onHigh(cb);
  return *this;
}

PinBuilder &PinBuilder::withDebounce(unsigned long ms)
{
  pin_.interruptDispatcher_.setDebounce(ms);
  return *this;
}

// ===== File: ./src/main.cpp =====
#include <Arduino.h>
#include <exercises.h>
#include <sandbox.h>

namespace entrypoint =
    // ex_w1
    // ex_w2_1_2
    // ex_w2_1_2_alt
    // ex_w3_3_4
    // ex_w3_5_6
    // ex_w4_4
    // ex_w4_5
    // ex_5_1
    // ex_5_2
    // ex_5_5
    // ex_5_6
    // ex_5_6_alt
    // ex_6_4
    // ex_7_3
    final
    // sandbox
    // Activate the exercise you want to run by uncommenting it
    ;

void setup()
{
  entrypoint::setup();
}

void loop()
{
  entrypoint::loop();
}

// === End of Combined File ===
