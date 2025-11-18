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

  stopMotors();
  encoder_.reset();

  targetSet_ = false;
  reachedTarget_ = false;

  encoder_.addTickCallback(makeCallback(this, &DistanceMotorDriver::onEncoderTick));
}

void DistanceMotorDriver::setTarget(float distanceCm, float power)
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
  {

    return;
  }

  driveStraight();
}

void DistanceMotorDriver::onEncoderTick()
{
  if (!targetSet_ || reachedTarget_)
    return;

  float distanceSinceStartCm = encoder_.getDistanceCm() - startDistanceCm_;
  if (distanceSinceStartCm >= targetDistanceCm_)
  {
    stopMotors();
    reachedTarget_ = true;
  }
}

void DistanceMotorDriver::driveStraight()
{
  leftMotor_.setPower(drivePower_);
  rightMotor_.setPower(drivePower_);

  leftMotor_.tick();
  rightMotor_.tick();
}

void DistanceMotorDriver::stopMotors()
{
  leftMotor_.stop();
  rightMotor_.stop();

  leftMotor_.tick();
  rightMotor_.tick();
}
