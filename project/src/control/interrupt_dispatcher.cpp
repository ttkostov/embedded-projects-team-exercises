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
