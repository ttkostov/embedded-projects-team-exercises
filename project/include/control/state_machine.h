
#pragma once
#include <Arduino.h>

template <typename Context>
struct IState
{
  virtual ~IState() {}
  virtual void onEnter(Context &ctx) {}
  virtual void onExit(Context &ctx) {}

  virtual void tick(Context &ctx) = 0;

  virtual void onEvent(Context &ctx, const String &ev) {}
};

template <typename Context>
class StateMachine
{
public:
  StateMachine(Context &ctx, IState<Context> &initial)
      : ctx_(ctx), current_(&initial)
  {
    // inject pointer so states can ask for transitions
    ctx_.stateMachine = this;
    current_->onEnter(ctx_);
  }

  void tick()
  {
    if (current_)
      current_->tick(ctx_);
  }

  void transitionTo(IState<Context> &next)
  {
    if (&next == current_)
      return;

    // exit current
    if (current_)
      current_->onExit(ctx_);

    // switch state
    previous_ = current_;
    current_ = &next;

    // enter next
    if (current_)
      current_->onEnter(ctx_);
  }

  IState<Context> *current() const { return current_; }

private:
  Context &ctx_;
  IState<Context> *current_ = nullptr;
  IState<Context> *previous_ = nullptr;
};
