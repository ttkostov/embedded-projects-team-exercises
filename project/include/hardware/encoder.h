#pragma once
#include <Arduino.h>
#include "pin.h"
#include "control/interrupt_dispatcher.h"

struct EncoderReading
{
  unsigned long ticks;
  float distanceCm;
};

class Encoder
{
public:
  Encoder(PinBuilder &pinBuilder, float encodingsPerCm);

  Encoder &begin();
  EncoderReading read() const;

  void reset();
  unsigned long getTicks() const;
  float getDistanceCm() const;

private:
  bool initialized_ = false;
  Pin &pin_;
  float encodingsPerCm_;

  volatile unsigned long tickCount_ = 0;

  void onEdge(); // ISR callback
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
