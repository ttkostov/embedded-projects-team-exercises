#include <hardware/car.h>

Car::Car(Joystick &joystick, Motor &leftMotor, Motor &rightMotor, IMotorDriver &motorDriver)
    : joystick_(joystick), leftMotor_(leftMotor), rightMotor_(rightMotor), motorDriver_(motorDriver) {}

void Car::begin()
{
  if (initialized_)
    return;

  joystick_.begin();
  leftMotor_.begin();
  rightMotor_.begin();
  motorDriver_.begin();

  initialized_ = true;
}

void Car::tick()
{
  // update motor state
  motorDriver_.tick();

  // apply motor state
  leftMotor_.tick();
  rightMotor_.tick();
}