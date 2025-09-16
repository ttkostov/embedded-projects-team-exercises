#include "hardware/pin.h"

Pin::Pin(uint8_t pinId)
    : id_(pinId), interruptDispatcher_(pinId) {}

Pin &Pin::begin()
{
  // configure mode
  pinMode(id_, mode_);

  // set initial value for outputs
  if (mode_ == OUTPUT)
  {
    digitalWrite(id_, initialValue_);
  }

  interruptDispatcher_.begin();
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
