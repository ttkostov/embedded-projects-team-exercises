#pragma once
#include <Arduino.h>
#include "control/interrupt_dispatcher.h"

class Pin
{
public:
  explicit Pin(uint8_t id);

  Pin &begin();

  uint8_t id();
  InterruptDispatcher &interruptDispatcher();

  void setHigh();
  void setLow();
  void setToggle();

  bool isHigh();
  bool isLow();
  int read();

  int analogReadValue();
  void analogWriteValue(int value);

private:
  bool initialized_ = false;

  uint8_t id_;
  uint8_t mode_ = INPUT;
  uint8_t initialValue_ = LOW;

  InterruptDispatcher interruptDispatcher_;

  friend class PinBuilder;
};

class PinBuilder
{
public:
  explicit PinBuilder(uint8_t id);

  Pin &build();

  PinBuilder &asInput();
  PinBuilder &asInputPullup();
  PinBuilder &asOutput();

  PinBuilder &withInitialValue(uint8_t value);

  PinBuilder &withFallingInterrupt(Callback cb);
  PinBuilder &withRisingInterrupt(Callback cb);
  PinBuilder &withChangeInterrupt(Callback cb);
  PinBuilder &withLowInterrupt(Callback cb);
  PinBuilder &withHighInterrupt(Callback cb);

  PinBuilder &withDebounce(unsigned long ms);

private:
  Pin pin_;
};
