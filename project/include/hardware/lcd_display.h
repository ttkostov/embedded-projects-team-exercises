#pragma once
#include <Arduino.h>
#include <LiquidCrystal.h>
#include "hardware/pin.h"

class LCDDisplay
{
public:
  LCDDisplay(Pin &registerSelect, Pin &enable,
             Pin &data4, Pin &data5, Pin &data6, Pin &data7,
             uint8_t cols, uint8_t rows);

  void begin();

  LiquidCrystal &lcd(); // access raw Arduino LCD if needed

  void printLine(uint8_t row, const String &text, bool clearRest = true);
  void updateRegion(uint8_t row, uint8_t col, const String &text);

  void centerText(uint8_t row, const String &text);
  void rightAlignText(uint8_t row, const String &text);
  void leftAlignText(uint8_t row, const String &text);

  void clearLine(uint8_t row);
  void printValue(const char *label, int value, uint8_t row);
  void printProgress(uint8_t row, int percent);

private:
  LiquidCrystal lcd_;
  uint8_t cols_;
  uint8_t rows_;

  String lineBuffer_[4]; // store last contents per row
};

class LCDDisplayBuilder
{
public:
  LCDDisplayBuilder &withRegisterSelect(PinBuilder pin);
  LCDDisplayBuilder &withEnable(PinBuilder pin);

  LCDDisplayBuilder &withData4(PinBuilder pin);
  LCDDisplayBuilder &withData5(PinBuilder pin);
  LCDDisplayBuilder &withData6(PinBuilder pin);
  LCDDisplayBuilder &withData7(PinBuilder pin);

  LCDDisplayBuilder &size(uint8_t cols, uint8_t rows);

  LCDDisplay &build();

private:
  Pin *registerSelect_ = nullptr;
  Pin *enable_ = nullptr;
  Pin *d4_ = nullptr;
  Pin *d5_ = nullptr;
  Pin *d6_ = nullptr;
  Pin *d7_ = nullptr;
  uint8_t cols_ = 16;
  uint8_t rows_ = 2;
};
