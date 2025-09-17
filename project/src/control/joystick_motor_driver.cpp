#include <control/motor_driver.h>

JoystickMotorDriver::JoystickMotorDriver(Motor &leftMotor, Motor &rightMotor, Joystick &joystick)
    : leftMotor_(leftMotor), rightMotor_(rightMotor), joystick_(joystick) {}

void JoystickMotorDriver::begin()
{
  if (initialized_)
    return;

  leftMotor_.begin();
  rightMotor_.begin();
  joystick_.begin();

  initialized_ = true;
}

void JoystickMotorDriver::tick()
{
  JoystickReading joystickState = joystick_.read();

  float leftPowerUnits = joystickState.yUnits + joystickState.xUnits;
  leftPowerUnits = constrain(leftPowerUnits, -1.0, 1.0);
  leftMotor_.setPower(leftPowerUnits);

  float rightPowerUnits = joystickState.yUnits - joystickState.xUnits;
  rightPowerUnits = constrain(rightPowerUnits, -1.0, 1.0);
  rightMotor_.setPower(rightPowerUnits);
}

JoystickVectorMotorDriver::JoystickVectorMotorDriver(Motor &leftMotor, Motor &rightMotor, Joystick &joystick)
    : leftMotor_(leftMotor), rightMotor_(rightMotor), joystick_(joystick) {}

void JoystickVectorMotorDriver::begin()
{
  if (initialized_)
    return;

  leftMotor_.begin();
  rightMotor_.begin();
  joystick_.begin();

  initialized_ = true;
}

void JoystickVectorMotorDriver::tick()
{
  // TODO: Implement relative driving logic. Compass needed.
}