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
