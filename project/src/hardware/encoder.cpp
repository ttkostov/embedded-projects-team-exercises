#include "hardware/encoder.h"

Encoder::Encoder(PinBuilder &pinBuilder, float encodingsPerCm)
    : pin_(pinBuilder
               .asInputPullup()
               .withChangeInterrupt(makeCallback(this, &Encoder::onEdge))
               .build()),
      encodingsPerCm_(encodingsPerCm)
{
}

Encoder &Encoder::begin()
{
  if (initialized_)
    return *this;

  initialized_ = true;

  pin_.begin();
  reset();

  return *this;
}

void Encoder::reset()
{
  noInterrupts();
  tickCount_ = 0;
  interrupts();
}

EncoderReading Encoder::read() const
{
  EncoderReading r;

  noInterrupts();
  r.ticks = tickCount_;
  interrupts();

  r.distanceCm = r.ticks / encodingsPerCm_;

  return r;
}

unsigned long Encoder::getTicks() const
{
  noInterrupts();
  unsigned long t = tickCount_;
  interrupts();

  return t;
}

float Encoder::getDistanceCm() const
{
  noInterrupts();
  unsigned long t = tickCount_;
  interrupts();

  return t / encodingsPerCm_;
}

void Encoder::onEdge()
{
  tickCount_++;
}
