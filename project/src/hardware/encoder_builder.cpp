#include "hardware/encoder.h"

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
  static Encoder encoder(pinBuilder_, encodingsPerCm_);
  return encoder;
}
