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