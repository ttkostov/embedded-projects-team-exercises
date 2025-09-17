#pragma once
#include <Arduino.h>
#include "hardware/motor.h"
#include "hardware/joystick.h"
#include "control/motor_driver.h"

class Car
{
public:
  Car(Joystick &joystick, Motor &leftMotor, Motor &rightMotor, IMotorDriver &motorDriver);

  void begin();
  void tick();

private:
  bool initialized_ = false;

  Joystick &joystick_;

  Motor &leftMotor_;
  Motor &rightMotor_;
  IMotorDriver &motorDriver_;
};