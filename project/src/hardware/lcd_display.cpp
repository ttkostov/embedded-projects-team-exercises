#include <Arduino.h>
#include "hardware/lcd_display.h"
#include <algorithm>

LCDDisplay::LCDDisplay(Pin &registerSelect, Pin &enable,
                       Pin &data4, Pin &data5, Pin &data6, Pin &data7,
                       uint8_t cols, uint8_t rows)
    : LiquidCrystal(registerSelect.id(), enable.id(),
                    data4.id(), data5.id(), data6.id(), data7.id()),
      cols_(cols), rows_(rows) {}

void LCDDisplay::begin()
{
  if (initialized_)
    return;

  LiquidCrystal::begin(cols_, rows_);
  for (uint8_t r = 0; r < rows_; r++)
  {
    lineBuffer_[r] = "";
  }

  initialized_ = true;
}

void LCDDisplay::clearLine(uint8_t row)
{
  if (row >= rows_)
    return;

  setCursor(0, row);
  for (uint8_t i = 0; i < cols_; i++)
  {
    print(' ');
  }
  setCursor(0, row);
  lineBuffer_[row] = String(cols_, ' ');
}

void LCDDisplay::printLine(uint8_t row, const String &text, bool clearRest)
{
  if (row >= rows_)
    return;

  // Avoid flicker: only update if different
  if (lineBuffer_[row] == text)
    return;

  setCursor(0, row);
  print(text);

  if (clearRest && text.length() < cols_)
  {
    for (uint8_t i = text.length(); i < cols_; i++)
    {
      print(' ');
    }
  }

  lineBuffer_[row] = text;
}

void LCDDisplay::updateRegion(uint8_t row, uint8_t col, const String &text)
{
  if (row >= rows_ || col >= cols_)
    return;

  setCursor(col, row);
  print(text);

  // update buffer only where new text goes
  String &buf = lineBuffer_[row];
  if (buf.length() < cols_)
    buf += String(' ', cols_ - buf.length());

  for (uint8_t i = 0; i < text.length() && (col + i) < cols_; i++)
  {
    buf[col + i] = text[i];
  }
}

void LCDDisplay::centerText(uint8_t row, const String &text)
{
  if (row >= rows_)
    return;
  int startCol = std::max(0, (cols_ - (int)text.length()) / 2);
  printLine(row, String(' ', startCol) + text, true);
}

void LCDDisplay::rightAlignText(uint8_t row, const String &text)
{
  if (row >= rows_)
    return;
  int startCol = std::max(0, (int)(cols_ - text.length()));
  printLine(row, String(' ', startCol) + text, true);
}

void LCDDisplay::leftAlignText(uint8_t row, const String &text)
{
  if (row >= rows_)
    return;
  printLine(row, text, true);
}

void LCDDisplay::printValue(const char *label, int value, uint8_t row)
{
  String text = String(label) + ": " + String(value);
  printLine(row, text);
}

void LCDDisplay::printProgress(uint8_t row, int percent)
{
  if (row >= rows_)
    return;
  int filled = (percent * cols_) / 100;
  String bar;
  for (int i = 0; i < filled; i++)
    bar += (char)255;
  for (int i = filled; i < cols_; i++)
    bar += ' ';
  printLine(row, bar, false);
}

// ----------------- LCDDisplayBuilder ----------------
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
