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
  tickCount_ = 0;
}

EncoderReading Encoder::read() const
{
  EncoderReading r;
  r.ticks = tickCount_;
  r.distanceCm = r.ticks / encodingsPerCm_;
  return r;
}

unsigned long Encoder::getTicks() const
{
  unsigned long t = tickCount_;
  return t;
}

float Encoder::getDistanceCm() const
{
  unsigned long t = tickCount_;
  return t / encodingsPerCm_;
}

bool Encoder::addTickCallback(const Callback &cb)
{
  if (callbackCount_ >= MAX_CALLBACKS)
    return false;

  callbacks_[callbackCount_++] = cb;
  return true;
}

void Encoder::onEdge() // ISR
{
  tickCount_++;

  // dispatch callbacks, must run quickly!
  for (uint8_t i = 0; i < callbackCount_; i++)
    callbacks_[i]();
}

// ----------------- EncoderBuilder ----------------

EncoderBuilder::EncoderBuilder(PinBuilder &pinBuilder)
    : pinBuilder_(pinBuilder)
{
}

EncoderBuilder &EncoderBuilder::withEncodingsPerCm(float encodingsPerCm)
{
  encodingsPerCm_ = encodingsPerCm;
  return *this;
}

Encoder &EncoderBuilder::build()
{
  return *(new Encoder(pinBuilder_, encodingsPerCm_));
}