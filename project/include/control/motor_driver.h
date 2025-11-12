#include <Arduino.h>
#include "hardware/motor.h"
#include "hardware/joystick.h"
#include "hardware/compass.h"
#include "util/directional.h"
#include "hardware/encoder.h"

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

class CompassHeadingMotorDriver : public IMotorDriver
{
public:
  CompassHeadingMotorDriver(Motor &leftMotor, Motor &rightMotor, Compass &compass);

  void begin() override;
  void tick() override;

  void setTolerance(const Angle &tolerance);
  void setTargetHeading(const Angle &heading);
  bool hasReachedTarget() const;

private:
  bool initialized_ = false;

  Motor &leftMotor_;
  Motor &rightMotor_;

  Compass &compass_;

  Angle targetHeading_;
  bool targetSet_ = false;
  Angle tolerance_ = Angle(5.0);

  bool approachTarget();
};

// Drives both motors straight based on a single encoder measurement.
class DistanceMotorDriver : public IMotorDriver
{
public:
  DistanceMotorDriver(Motor &leftMotor, Motor &rightMotor, Encoder &encoder);

  void begin() override;
  void tick() override;

  // Configure a target distance (in cm) and power (-1.0 - 1.0)
  void setTarget(float distanceCm, float power = 0.5f);
  bool hasReachedTarget() const;

  void reset();

private:
  Motor &leftMotor_;
  Motor &rightMotor_;
  Encoder &encoder_;

  bool initialized_ = false;
  bool targetSet_ = false;
  bool reachedTarget_ = false;

  float targetDistanceCm_ = 0.0f;
  float startDistanceCm_ = 0.0f;
  float drivePower_ = 0.5f;

  void driveStraight();
  void stopMotors();
};