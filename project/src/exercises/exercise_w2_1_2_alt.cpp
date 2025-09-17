#include <LiquidCrystal.h>
#include <Arduino.h>
#include "exercises.h"
#include "hardware.h"
#include "physical.h"

namespace ex_w2_1_2_alt
{

  namespace p = physical;

  void exerciseOne();
  void printCoordinate(int value, int rowIndex, String coordinate);
  void calculatePercentOfCoordinateAndPrint(int value, int rowIndex);

  void exerciseTwo();
  void onButtonPressed();

  unsigned long counter = 0;
  volatile bool toggleScreen = false;

  void setup()
  {
    // put your setup code here, to run once:
    Serial.begin(9600);

    p::joystick::device.onPress(makeCallback(onButtonPressed));
    p::joystick::device.begin();

    p::lcd::device.begin();
  }

  void loop()
  {
    p::lcd::device.clear();

    toggleScreen ? exerciseOne() : exerciseTwo();
    delay(200);
  }

  void exerciseTwo()
  {

    p::lcd::device.setCursor(0, 0);
    p::lcd::device.print("Push counter:");
    p::lcd::device.setCursor(0, 1);
    p::lcd::device.print(counter);
  }

  void onButtonPressed()
  {
    Serial.print("Button pressed! ");
    toggleScreen = !toggleScreen;
    Serial.println(++counter);
  }

  void exerciseOne()
  {
    int xValue = p::joystick::device.readX();
    int yValue = 1023 - p::joystick::device.readY(); // correct direction for the board

    printCoordinate(xValue, 0, "X");
    printCoordinate(yValue, 1, "Y");
    calculatePercentOfCoordinateAndPrint(xValue, 0);
    calculatePercentOfCoordinateAndPrint(yValue, 1);
  }

  void printCoordinate(int value, int rowIndex, String coordinate)
  {
    p::lcd::device.setCursor(0, rowIndex);
    p::lcd::device.print(coordinate);
    p::lcd::device.print(":");

    p::lcd::device.print("    ");
    p::lcd::device.setCursor(2, rowIndex);
    p::lcd::device.print(value);
  }

  void calculatePercentOfCoordinateAndPrint(int value, int rowIndex)
  {
    float percentage = (value) / 1023.0 * 100.0;

    p::lcd::device.setCursor(18, rowIndex);
    p::lcd::device.print("%");

    p::lcd::device.setCursor(15, rowIndex);
    p::lcd::device.print("   ");

    p::lcd::device.setCursor(15, rowIndex);
    p::lcd::device.print((int)(percentage));
  }

}