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
