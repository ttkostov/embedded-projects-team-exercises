#include <LiquidCrystal.h>
#include <Arduino.h>
#include "exercises.h"
#include "hardware.h"
#include "physical.h"

namespace ex_w4_4
{

  namespace p = physical;

  void startMotor();
  void stopMotor();

  void exerciseOne();
  void printCoordinate(int value, int rowIndex, String coordinate);
  void calculatePercentOfCoordinateAndPrint(int value, int rowIndex);

  void exerciseTwo();
  void onButtonPressed();

  unsigned long counter = 0;

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

    p::motor::encoderLeft.asInputPullup().withChangeInterrupt(makeCallback(
                                                                  []()
                                                                  {
if(countEncodings) counter++; }))
        .build()
        .begin();

    p::lcd::device.begin();
  }

  void loop()
  {
    // p::lcd::device.clear();

    // toggleScreen ? exerciseOne() : exerciseTwo();

    if (millis() > 5000 && !done)
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

    if (millis() - runStartTime > 4000 && countEncodings)
    {
      stopMotor();
      countEncodings = false;

      Serial.println("Final counter: " + String(counter) + " in time: " + String(millis() - runStartTime) + " ms");
    }

    delay(200);
  }

  void exerciseTwo()
  {

    p::lcd::device.setCursor(0, 0);
    p::lcd::device.print("Push counter:");
    p::lcd::device.setCursor(0, 1);
    p::lcd::device.print(counter);
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