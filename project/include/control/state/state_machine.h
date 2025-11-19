#pragma once
#include <Arduino.h>

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
  virtual void onEvent(Context &ctx, const Event &ev) {};
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
      Serial.println(F("[StateMachine] ERROR: Stack overflow!"));
      return;
    }

    if (stackSize_ > 0)
    {
      Serial.println("[" + String(stack_[stackSize_ - 1]->name) +
                     "]: Pushing -> " + next.name);
      stack_[stackSize_ - 1]->onExit(ctx_);
    }

    stack_[stackSize_++] = &next;

    Serial.println("[" + String(next.name) + "]: Enter");
    next.onEnter(ctx_);
  }

  void popState()
  {
    if (stackSize_ == 0)
      return;

    IState<Context> *curr = stack_[stackSize_ - 1];

    Serial.println("[" + String(curr->name) + "]: Pop (Exit)");
    curr->onExit(ctx_);

    stackSize_--;

    if (stackSize_ > 0)
    {
      Serial.println("[" + String(stack_[stackSize_ - 1]->name) + "]: Resume");
      stack_[stackSize_ - 1]->onResume(ctx_);
    }
  }

  void handleEvent(const Event &ev)
  {
    if (stackSize_ == 0)
      return;

    IState<Context> *curr = stack_[stackSize_ - 1];

    Serial.println("[" + String(curr->name) + "]: Sending event '" + ev.name() + "'");
    curr->onEvent(ctx_, ev);
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