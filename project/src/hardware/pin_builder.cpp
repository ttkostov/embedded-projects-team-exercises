#include "hardware/pin.h"

PinBuilder::PinBuilder(uint8_t id)
    : pin_(id) {}

Pin &PinBuilder::build()
{
  return pin_;
}

PinBuilder &PinBuilder::asInput()
{
  pin_.mode_ = INPUT;
  return *this;
}

PinBuilder &PinBuilder::asInputPullup()
{
  pin_.mode_ = INPUT_PULLUP;
  return *this;
}

PinBuilder &PinBuilder::asOutput()
{
  pin_.mode_ = OUTPUT;
  return *this;
}

PinBuilder &PinBuilder::withInitialValue(uint8_t value)
{
  pin_.initialValue_ = value;
  return *this;
}

PinBuilder &PinBuilder::withFallingInterrupt(Callback cb)
{
  pin_.interruptDispatcher_.onFalling(cb);
  return *this;
}

PinBuilder &PinBuilder::withRisingInterrupt(Callback cb)
{
  pin_.interruptDispatcher_.onRising(cb);
  return *this;
}

PinBuilder &PinBuilder::withChangeInterrupt(Callback cb)
{
  pin_.interruptDispatcher_.onChange(cb);
  return *this;
}

PinBuilder &PinBuilder::withLowInterrupt(Callback cb)
{
  pin_.interruptDispatcher_.onLow(cb);
  return *this;
}

PinBuilder &PinBuilder::withHighInterrupt(Callback cb)
{
  pin_.interruptDispatcher_.onHigh(cb);
  return *this;
}

PinBuilder &PinBuilder::withDebounce(unsigned long ms)
{
  pin_.interruptDispatcher_.setDebounce(ms);
  return *this;
}
