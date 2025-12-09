#include <LiquidCrystal.h>
#include <Arduino.h>
#include "exercises.h"
#include "hardware.h"
#include "physical.h"

/*
1754, 4001ms, 65cm
1755, 4001ms, 65.5cm
1742, 4001ms, 65.5cm

weight:
l: 1556, 52cm -> 29.9230
l: 1601, 58cm -> 27.603

r: 1586, 57.8cm -> 27.4394
r: 1577, 57cm -> 27.6666


left=1590,27.226 right=1566,26.815 58.4
left=1593,27.277 right=1576,26.986 58.4
left=1564,27.342 right=1563,27.325 57.2
left=1606,27.037 right=1585,26.683 59.4
*/

namespace ex_w4_5
{

  namespace p = physical;

  void startMotor();
  void stopMotor();

  void exerciseOne();
  void printCoordinate(int value, int rowIndex, String coordinate);
  void calculatePercentOfCoordinateAndPrint(int value, int rowIndex);

  void exerciseTwo();
  void onButtonPressed();

  float targetDistanceCm = 100.5;
  float encodingsPerCm = 27.2305;
  float targetEncodings = targetDistanceCm * encodingsPerCm;

  unsigned long leftCounter = 0;
  unsigned long rightCounter = 0;

  volatile bool toggleScreen = true;
  volatile bool runMotor = false;
  volatile long runStartTime = 0;
  volatile bool countEncodings = false;

  volatile bool done = false;

  void setup()
  {
    // put your setup code here, to run once:
    Serial.begin(9600);

    p::joystick::device.onPress(makeCallback(onButtonPressed));
    p::joystick::device.begin();

    p::motor::encoderLeft
        .asInputPullup()
        .withChangeInterrupt(
            makeCallback(
                []()
                {
                  if (countEncodings)
                    leftCounter++;

                  if (leftCounter >= targetEncodings && countEncodings)
                  {
                    stopMotor();
                    countEncodings = false;

                    Serial.println("Final counter: left=" + String(leftCounter) + " right=" + String(rightCounter) + "in time: " + String(millis() - runStartTime) + " ms");
                  }
                }))
        .build()
        .begin();

    p::motor::encoderRight.asInputPullup().withChangeInterrupt(makeCallback(
                                                                   []()
                                                                   {
    if(countEncodings) rightCounter++; }))
        .build()
        .begin();

    p::lcd::device.begin();
  }

  void loop()
  {

    exerciseTwo();
    // p::lcd::device.clear();

    // toggleScreen ? exerciseOne() : exerciseTwo();

    if (millis() > 2000 && !done)
    {
      runMotor = true;
      done = true;
    }

    if (runMotor)
    {
      countEncodings = true;
      startMotor();
      runStartTime = millis();
      runMotor = false;
    }

    delay(200);
  }

  void exerciseTwo()
  {
    p::lcd::device.printLine(0, "Left:" + String(leftCounter));
    p::lcd::device.printLine(1, "Right:" + String(rightCounter));
  }

  void startMotor()
  {

#define Motor_forward 0
#define Motor_return 1
#define Motor_L_dir_pin 7
#define Motor_R_dir_pin 8
#define Motor_L_pwm_pin 9
#define Motor_R_pwm_pin 10

    analogWrite(Motor_L_pwm_pin, 1023);
    analogWrite(Motor_R_pwm_pin, 1023);

    digitalWrite(Motor_R_dir_pin, Motor_return);
    digitalWrite(Motor_L_dir_pin, Motor_return);
  }

  void stopMotor()
  {

#define Motor_forward 0
#define Motor_return 1
#define Motor_L_dir_pin 7
#define Motor_R_dir_pin 8
#define Motor_L_pwm_pin 9
#define Motor_R_pwm_pin 10

    analogWrite(Motor_L_pwm_pin, 0);
    analogWrite(Motor_R_pwm_pin, 0);
  }

  void onButtonPressed()
  {
    runMotor = true;
  }

  void exerciseOne()
  {
    JoystickReading reading = p::joystick::device.read();
    int xValue = map(reading.xUnits * 100, -100, 100, 0, 1023);
    int yValue = map(reading.yUnits * 100, -100, 100, 0, 1023);

    printCoordinate(xValue, 0, "X");
    printCoordinate(yValue, 1, "Y");

    Serial.println("X: " + String(xValue) + " Y: " + String(yValue));
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