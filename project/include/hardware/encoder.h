#pragma once
#include <Arduino.h>
#include "pin.h"
#include "control/interrupt_dispatcher.h"
#include "control/callback.h"

struct EncoderReading
{
  unsigned long ticks;
  float distanceCm;
};

class Encoder
{
public:
  static constexpr uint8_t MAX_CALLBACKS = 4;

  Encoder(PinBuilder &pinBuilder, float encodingsPerCm);

  Encoder &begin();
  EncoderReading read() const;

  void reset();
  unsigned long getTicks() const;
  float getDistanceCm() const;

  bool addTickCallback(const Callback &cb);

private:
  bool initialized_ = false;
  Pin &pin_;
  float encodingsPerCm_;

  volatile unsigned long tickCount_ = 0;

  Callback callbacks_[MAX_CALLBACKS];
  uint8_t callbackCount_ = 0;

  void onEdge();
};

class EncoderBuilder
{
public:
  EncoderBuilder(PinBuilder &pinBuilder);

  EncoderBuilder &withEncodingsPerCm(float encodingsPerCm);
  Encoder &build();

private:
  PinBuilder &pinBuilder_;
  float encodingsPerCm_ = 27.2305f;
};