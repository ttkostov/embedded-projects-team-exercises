#pragma once
#include <Arduino.h>
#include <vector>

/*
  Replacement for std::function for AVR.
*/
struct Callback
{
  using Thunk = void (*)(void *);

  void *ctx;        // object pointer, or nullptr
  Thunk fn;         // trampoline
  void (*freeFn)(); // optional free/static function pointer

  void operator()() const
  {
    if (fn)
      fn(ctx);
  }

  // Trampoline for free/static functions
  static void callFree(void *ctx)
  {
    Callback *cb = static_cast<Callback *>(ctx);
    if (cb->freeFn)
      cb->freeFn();
  }
};

// Member function binder
template <typename T>
Callback makeCallback(T *obj, void (T::*method)())
{
  struct MethodHelper
  {
    static void call(void *ctx)
    {
      T *self = static_cast<T *>(ctx);
      (self->*method_)();
    }
    static void (T::*method_)();
  };
  MethodHelper::method_ = method;
  return {obj, &MethodHelper::call, nullptr};
}

// Free/static function binder
inline Callback makeCallback(void (*fn)())
{
  Callback cb;
  cb.ctx = &cb; // so trampoline can get back to us
  cb.fn = &Callback::callFree;
  cb.freeFn = fn;
  return cb;
}

class InterruptDispatcher
{
public:
  explicit InterruptDispatcher(uint8_t id, int lastState = LOW);

  void begin(int mode = CHANGE); // registers interrupt
  void handleInterrupt();

  void setDebounce(unsigned long ms);

  void onLow(Callback cb);
  void onHigh(Callback cb);
  void onChange(Callback cb);
  void onRising(Callback cb);
  void onFalling(Callback cb);

  static InterruptDispatcher *dispatchers_[NUM_DIGITAL_PINS];

private:
  uint8_t pinId_;
  unsigned long debounceMs_ = 0;
  unsigned long lastInterruptTime_ = 0;
  int lastState_ = LOW;

  std::vector<Callback> lowHandlers_;
  std::vector<Callback> highHandlers_;
  std::vector<Callback> changeHandlers_;
  std::vector<Callback> risingHandlers_;
  std::vector<Callback> fallingHandlers_;
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
