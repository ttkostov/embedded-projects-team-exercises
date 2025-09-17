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
