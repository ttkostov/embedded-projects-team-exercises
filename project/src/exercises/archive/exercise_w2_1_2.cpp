#include <LiquidCrystal.h>
#include <Arduino.h>
#include "exercises.h"

namespace ex_w2_1_2
{

  void exerciseOne();
  void printCoordinate(int value, int rowIndex, String coordinate);
  void calculatePercentOfCoordinateAndPrint(int value, int rowIndex);

  void exerciseTwo();
  void onButtonPressed();

  const int rs = 37,
            en = 36, d4 = 35, d5 = 34, d6 = 33, d7 = 32;
  LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

  volatile unsigned long lastInterrupted = 0;
  unsigned long counter = 0;
  volatile bool toggleScreen = false;

  void setup()
  {
    // put your setup code here, to run once:
    Serial.begin(9600);
    pinMode(19, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(19), onButtonPressed, FALLING);

    lcd.begin(20, 4);
  }

  void loop()
  {
    lcd.clear();
    toggleScreen ? exerciseOne() : exerciseTwo();
    delay(200);
  }

  void exerciseTwo()
  {

    lcd.setCursor(0, 0);
    lcd.print("Push counter:");
    lcd.setCursor(0, 1);
    lcd.print(counter);
  }

  void onButtonPressed()
  {
    unsigned long now = millis();
    if (now - lastInterrupted > 200)
    {
      toggleScreen = !toggleScreen;
      Serial.println(++counter);
      lastInterrupted = now;
    }
  }

  void exerciseOne()
  {
    int xValue = analogRead(A7);
    int yValue = 1023 - analogRead(A8); // correct direction for the board

    printCoordinate(xValue, 0, "X");
    printCoordinate(yValue, 1, "Y");
    calculatePercentOfCoordinateAndPrint(xValue, 0);
    calculatePercentOfCoordinateAndPrint(yValue, 1);
  }

  void printCoordinate(int value, int rowIndex, String coordinate)
  {
    lcd.setCursor(0, rowIndex);
    lcd.print(coordinate);
    lcd.print(":");

    lcd.print("    ");
    lcd.setCursor(2, rowIndex);
    lcd.print(value);
  }

  void calculatePercentOfCoordinateAndPrint(int value, int rowIndex)
  {
    float percentage = (value) / 1023.0 * 100.0;

    lcd.setCursor(18, rowIndex);
    lcd.print("%");

    lcd.setCursor(15, rowIndex);
    lcd.print("   ");

    lcd.setCursor(15, rowIndex);
    lcd.print((int)(percentage));
  }

}