#include <Arduino.h>
#include <hardware/motor.h>
#include <hardware/joystick.h>

class IMotorDriver
{
public:
  virtual void begin() = 0;
  virtual void tick() = 0;
};

class JoystickMotorDriver : public IMotorDriver
{
public:
  JoystickMotorDriver(Motor &leftMotor, Motor &rightMotor, Joystick &joystick);

  void begin() override;
  void tick() override;

private:
  bool initialized_ = false;

  Motor &leftMotor_;
  Motor &rightMotor_;

  Joystick &joystick_;
};

class JoystickVectorMotorDriver : public IMotorDriver
{
public:
  JoystickVectorMotorDriver(Motor &leftMotor, Motor &rightMotor, Joystick &joystick);

  void begin() override;
  void tick() override;

private:
  bool initialized_ = false;

  Motor &leftMotor_;
  Motor &rightMotor_;

  Joystick &joystick_;
};