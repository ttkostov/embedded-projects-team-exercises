#include "control/motor_driver.h"

DistanceMotorDriver::DistanceMotorDriver(Motor &leftMotor, Motor &rightMotor, Encoder &encoder)
    : leftMotor_(leftMotor),
      rightMotor_(rightMotor),
      encoder_(encoder)
{
}

void DistanceMotorDriver::begin()
{
  if (initialized_)
    return;
  initialized_ = true;

  leftMotor_.stop();
  rightMotor_.stop();
  encoder_.reset();

  targetSet_ = false;
  reachedTarget_ = false;
}

void DistanceMotorDriver::setTargetDistance(float distanceCm, float power)
{
  if (!initialized_)
    return;

  targetDistanceCm_ = distanceCm;
  drivePower_ = constrain(power, -1.0f, 1.0f);

  startDistanceCm_ = encoder_.getDistanceCm();
  targetSet_ = true;
  reachedTarget_ = false;
}

void DistanceMotorDriver::reset()
{
  targetSet_ = false;
  reachedTarget_ = false;
  encoder_.reset();
}

bool DistanceMotorDriver::hasReachedTarget() const
{
  return targetSet_ && reachedTarget_;
}

void DistanceMotorDriver::tick()
{
  if (!initialized_ || !targetSet_ || reachedTarget_)
    return;

  // How far we've gone since starting
  float distanceCm = encoder_.getDistanceCm() - startDistanceCm_;

  if (distanceCm >= targetDistanceCm_)
  {
    stopMotors();
    reachedTarget_ = true;
    return;
  }

  driveStraight();
}

void DistanceMotorDriver::driveStraight()
{
  leftMotor_.setPower(drivePower_);
  rightMotor_.setPower(drivePower_);
}

void DistanceMotorDriver::stopMotors()
{
  leftMotor_.stop();
  rightMotor_.stop();
}
