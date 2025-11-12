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
  virtual void onExit(Context &ctx) {};
  virtual void onEvent(Context &ctx, const Event &ev) {};
  virtual void tick(Context &ctx) {};
};

template <typename Context>
class StateMachine
{
public:
  StateMachine(Context &ctx, IState<Context> &initial)
      : ctx_(ctx), current_(&initial)
  {
    ctx_.stateMachine = this;

    Serial.println("[" + String(current_->name) + "]: Used as initial state");
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

    if (current_)
    {
      Serial.println("[" + String(current_->name) + "]: Exiting...");
      current_->onExit(ctx_);
    }

    previous_ = current_;
    current_ = &next;

    if (current_)
    {
      Serial.println("[" + String(current_->name) + "]: Entering...");
      current_->onEnter(ctx_);
    }
  }

  IState<Context> *current() const { return current_; }

  void handleEvent(const Event &ev)
  {
    if (current_)
    {
      Serial.println("[" + String(current_->name) + "]: Sending event '" + String(ev.name()) + "'");
      current_->onEvent(ctx_, ev);
    }
  }

private:
  Context &ctx_;
  IState<Context> *current_ = nullptr;
  IState<Context> *previous_ = nullptr;
};
