#include <Arduino.h>
#include "hardware/lcd_display.h"
#include <algorithm>

LCDDisplay::LCDDisplay(Pin &registerSelect, Pin &enable,
                       Pin &data4, Pin &data5, Pin &data6, Pin &data7,
                       uint8_t cols, uint8_t rows)
    : lcd_(registerSelect.id(), enable.id(),
           data4.id(), data5.id(), data6.id(), data7.id()),
      cols_(cols), rows_(rows) {}

void LCDDisplay::begin()
{
  lcd_.begin(cols_, rows_);
  for (uint8_t r = 0; r < rows_; r++)
  {
    lineBuffer_[r] = "";
  }
}

LiquidCrystal &LCDDisplay::lcd()
{
  return lcd_;
}

void LCDDisplay::clearLine(uint8_t row)
{
  lcd_.setCursor(0, row);
  for (uint8_t i = 0; i < cols_; i++)
  {
    lcd_.print(' ');
  }
  lcd_.setCursor(0, row);
  lineBuffer_[row] = String(cols_, ' ');
}

void LCDDisplay::printLine(uint8_t row, const String &text, bool clearRest)
{
  if (row >= rows_)
    return;

  // Avoid flicker: only update if different
  if (lineBuffer_[row] == text)
    return;

  lcd_.setCursor(0, row);
  lcd_.print(text);

  if (clearRest && text.length() < cols_)
  {
    for (uint8_t i = text.length(); i < cols_; i++)
    {
      lcd_.print(' ');
    }
  }

  lineBuffer_[row] = text;
}

void LCDDisplay::updateRegion(uint8_t row, uint8_t col, const String &text)
{
  if (row >= rows_ || col >= cols_)
    return;

  lcd_.setCursor(col, row);
  lcd_.print(text);

  // update buffer only where new text goes
  String &buf = lineBuffer_[row];
  if (buf.length() < cols_)
    buf = buf + String(' ', cols_ - buf.length());

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
  int startCol = std::max(0, cols_ - (int)text.length());
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
