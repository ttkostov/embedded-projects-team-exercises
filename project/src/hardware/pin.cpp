#include "hardware/pin.h"

Pin::Pin(uint8_t pinId)
    : id_(pinId), interruptDispatcher_(pinId) {}

Pin &Pin::begin()
{
  if (initialized_)
    return *this;

  // configure mode
  pinMode(id_, mode_);

  // set initial value for outputs
  if (mode_ == OUTPUT)
  {
    digitalWrite(id_, initialValue_);
  }

  interruptDispatcher_.begin();

  initialized_ = true;
  return *this;
}

uint8_t Pin::id() { return id_; }

InterruptDispatcher &Pin::interruptDispatcher()
{
  return interruptDispatcher_;
}

void Pin::setHigh() { digitalWrite(id_, HIGH); }
void Pin::setLow() { digitalWrite(id_, LOW); }
void Pin::setToggle() { digitalWrite(id_, isHigh() ? LOW : HIGH); }

bool Pin::isHigh() { return read() == HIGH; }
bool Pin::isLow() { return read() == LOW; }
int Pin::read() { return digitalRead(id_); }

int Pin::analogReadValue() { return analogRead(id_); }
void Pin::analogWriteValue(int v) { analogWrite(id_, v); }

// ----------------- PinBuilder ----------------

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
