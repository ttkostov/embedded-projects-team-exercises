#include "control/motor_driver.h"

CompassHeadingMotorDriver::CompassHeadingMotorDriver(
    Motor &left, Motor &right, Compass &compass)
    : leftMotor_(left), rightMotor_(right), compass_(compass)
{
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

void CompassHeadingMotorDriver::reset()
{
  targetSet_ = false;
  reachedTarget_ = false;
  stopMotors();
}

void CompassHeadingMotorDriver::setTargetHeading(const Angle &heading)
{
  targetHeading_ = heading;
  targetSet_ = true;
  reachedTarget_ = false;
}

void CompassHeadingMotorDriver::setTolerance(const Angle &tolerance)
{
  tolerance_ = tolerance;
}

bool CompassHeadingMotorDriver::hasReachedTarget() const
{
  // reached when turning is complete AND no active target is set
  return reachedTarget_ && !targetSet_;
}

void CompassHeadingMotorDriver::tick()
{
  if (!initialized_)
    return;

  // stop motors if no target is set
  if (!targetSet_)
  {
    stopMotors();
    return;
  }

  // perform turning towards target
  if (approachTarget())
  {
    reachedTarget_ = true;
    targetSet_ = false;

    Serial.println("Reached target heading.");

    stopMotors();
  }

  leftMotor_.tick();
  rightMotor_.tick();
}

bool CompassHeadingMotorDriver::approachTarget()
{
  CompassReading reading = compass_.read();
  Angle currentHeading = reading.heading;

  Angle diff = targetHeading_.differenceTo(currentHeading);
  float headingError = diff.value();
  float absErr = abs(headingError);

  // check if within tolerance
  if (absErr <= tolerance_.value())
  {
    return true;
  }

  // small error -> small power
  // large error -> quadratically larger power
  float wantedPower = (absErr * absErr) / 2000.0f;
  float power = constrain(wantedPower, 0.2f, 1.0f);

  if (headingError > 0)
  {
    // turn left
    leftMotor_.setPower(-power);
    rightMotor_.setPower(power);
  }
  else
  {
    // turn right
    leftMotor_.setPower(power);
    rightMotor_.setPower(-power);
  }

  return false; // not finished turning
}

void CompassHeadingMotorDriver::stopMotors()
{
  leftMotor_.stop();
  rightMotor_.stop();

  leftMotor_.tick();
  rightMotor_.tick();
}
