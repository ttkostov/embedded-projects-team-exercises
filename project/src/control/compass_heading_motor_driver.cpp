#include "control/motor_driver.h"

CompassHeadingMotorDriver::CompassHeadingMotorDriver(Motor &left, Motor &right, Compass &compass)
    : leftMotor_(left), rightMotor_(right), compass_(compass) {}

void CompassHeadingMotorDriver::setTargetHeading(const Angle &heading)
{
  targetHeading_ = heading;
  targetSet_ = true;
}

void CompassHeadingMotorDriver::setTolerance(const Angle &tolerance)
{
  tolerance_ = tolerance;
}

bool CompassHeadingMotorDriver::hasReachedTarget() const
{
  return targetSet_;
}

void CompassHeadingMotorDriver::begin()
{
  if (initialized_)
    return;

  leftMotor_.begin();
  rightMotor_.begin();
  compass_.begin();

  initialized_ = true;
}

void CompassHeadingMotorDriver::tick()
{
  if (!targetSet_)
    return;

  if (approachTarget())
  {
    targetSet_ = false;
  }
}

bool CompassHeadingMotorDriver::approachTarget()
{
  CompassReading reading = compass_.read();
  Angle currentHeading = reading.heading;
  float headingError = targetHeading_.value() - currentHeading.value();

  if (abs(headingError) <= tolerance_.value())
  {
    leftMotor_.stop();
    rightMotor_.stop();
    return true;
  }

  // Use a quadratic function to determine power based on heading error
  float wantedPower = (abs(headingError) * abs(headingError)) / 2000.0;
  float power = std::max(std::min(1.0f, wantedPower), 0.2f);

  if (headingError > 0)
  {
    leftMotor_.setPower(-power);
    rightMotor_.setPower(power);
  }
  else
  {
    leftMotor_.setPower(power);
    rightMotor_.setPower(-power);
  }
  return false;
}