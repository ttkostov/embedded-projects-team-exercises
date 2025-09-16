#include "hardware/lcd_display.h"

LCDDisplayBuilder &LCDDisplayBuilder::withRegisterSelect(PinBuilder pin)
{
  registerSelect_ = &pin.build();
  return *this;
}

LCDDisplayBuilder &LCDDisplayBuilder::withEnable(PinBuilder pin)
{
  enable_ = &pin.asOutput().build();
  return *this;
}

LCDDisplayBuilder &LCDDisplayBuilder::withData4(PinBuilder pin)
{
  d4_ = &pin.asOutput().build();
  return *this;
}

LCDDisplayBuilder &LCDDisplayBuilder::withData5(PinBuilder pin)
{
  d5_ = &pin.asOutput().build();
  return *this;
}

LCDDisplayBuilder &LCDDisplayBuilder::withData6(PinBuilder pin)
{
  d6_ = &pin.asOutput().build();
  return *this;
}

LCDDisplayBuilder &LCDDisplayBuilder::withData7(PinBuilder pin)
{
  d7_ = &pin.asOutput().build();
  return *this;
}

LCDDisplayBuilder &LCDDisplayBuilder::size(uint8_t cols, uint8_t rows)
{
  cols_ = cols;
  rows_ = rows;
  return *this;
}

LCDDisplay &LCDDisplayBuilder::build()
{
  static LCDDisplay lcd(*registerSelect_, *enable_, *d4_, *d5_, *d6_, *d7_, cols_, rows_);
  return lcd;
}
