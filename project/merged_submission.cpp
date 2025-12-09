// === Combined C++ Submission ===
// Generated on Tue Dec  9 01:16:27 PM EET 2025
// Source root: ./src

// === Header Files ===


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

// void setup()
// {
//   entrypoint::setup();

//   Serial.begin(9600);  // USB serial to PC
//   Serial1.begin(9600); // ESP link
// }

// void loop()
// {
//   // entrypoint::loop();

//   // From ESP to PC
//   if (Serial1.available())
//   {
//     int ch = Serial1.read();
//     Serial.write(ch);
//   }

//   // From PC to ESP
//   if (Serial.available())
//   {
//     int ch = Serial.read();
//     Serial1.write(ch);
//   }
// }


// === End of Combined File ===
