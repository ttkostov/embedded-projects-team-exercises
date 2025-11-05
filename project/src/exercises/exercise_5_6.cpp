#include <Arduino.h>
#include <algorithm>
#include <LiquidCrystal.h>
#include "exercises.h"
#include "hardware.h"
#include "physical.h"
#include <Wire.h>
namespace ex_5_6
{

  enum class ProgramState
  {
    IDLE,
    LEG,
    TURN,
  };

  int executedSteps = 0;

  ProgramState previousState = ProgramState::IDLE;
  ProgramState state = ProgramState::IDLE;

#define CMPS14_address 0x60 // I2C slave address for COMPS module, default:0xC0 >> 0x60

  namespace p = physical;

  float maxPower = 255;

  float stage = 0;

  float targetHeading = 0;

  // --------------------

  float encodingsPerCm = 27.2305;
  float targetEncodings;

  unsigned long leftCounter = 0;
  unsigned long rightCounter = 0;

  volatile bool toggleScreen = true;
  volatile bool runMotor = false;
  volatile long runStartTime = 0;
  volatile bool countEncodings = false;

  volatile bool distanceDriven = true;
  volatile bool do_run = false;

  volatile bool done = false;

  void startMotor(float power);
  void startMotorLeft(float power);
  void startMotorRight(float power);
  void stopMotor();

  void driveDistance(float distanceCm, float power)
  {
    Serial.println("Driving distance: " + String(distanceCm) + " cm with power: " + String(power));

    leftCounter = 0;
    startMotor(power);
    runStartTime = millis();
    distanceDriven = false;

    targetEncodings = distanceCm * encodingsPerCm;
  }

  void onButtonPressed()
  {
    if (state == ProgramState::IDLE)
    {
      state = ProgramState::LEG;
      executedSteps = 0;

      distanceDriven = true;
      do_run = true;
    }
  }

  void setup()
  {
    // put your setup code here, to run once:
    Serial.begin(9600);

    p::joystick::device.onPress(makeCallback(onButtonPressed));
    p::joystick::device.begin();

    p::motor::encoderLeft
        .asInputPullup()
        .withChangeInterrupt(
            makeCallback(
                []()
                {
                  leftCounter++;

                  if (distanceDriven == false && leftCounter >= targetEncodings)
                  {
                    stopMotor();
                    distanceDriven = true;
                  }
                }))
        .build()
        .begin();

    p::motor::encoderRight.asInputPullup().withChangeInterrupt(makeCallback(
                                                                   []()
                                                                   {
    if(countEncodings) rightCounter++; }))
        .build()
        .begin();

    p::lcd::device.begin();

    Wire.begin();
  }

  float wrapAngle(float angle)
  {
    angle = fmod(angle, 360.0);
    if (angle < 0)
      angle += 360.0;
    return angle;
  }

  float turnAngleLeft(float currentAngle, float degrees)
  {
    return wrapAngle(currentAngle - degrees);
  }

  float turnAngleRight(float currentAngle, float degrees)
  {
    return wrapAngle(currentAngle + degrees);
  }

  byte readByte(byte address, byte reg)
  {
    Wire.beginTransmission(address);
    Wire.write(reg);
    Wire.endTransmission(false);

    Wire.requestFrom(address, 1, (int)true);

    if (Wire.available())
    {
      return Wire.read();
    }
    return 0;
  }

  float offsetDegrees(float degrees, float offsetDegrees)
  {
    float adjustedDegrees = degrees + offsetDegrees;
    if (adjustedDegrees >= 360.0)
    {
      adjustedDegrees -= 360.0;
    }
    else if (adjustedDegrees < 0.0)
    {
      adjustedDegrees += 360.0;
    }
    return adjustedDegrees;
  }

  float readHeadingDegrees()
  {
    byte high_byte = readByte(CMPS14_address, 2);
    byte low_byte = readByte(CMPS14_address, 3);

    int heading = (high_byte << 8) | low_byte; // Combine high and low bytes

    return heading / 10.0; // Convert to degrees (range is 0-3599 -> 0.0-359.9 degrees)
  }

  String getHeadingDirection(float degrees)
  {
    degrees = fmod(degrees, 360.0);
    if (degrees < 0)
      degrees += 360.0;

    if (degrees >= 23 && degrees < 68)
      return "NE";
    else if (degrees >= 68 && degrees < 113)
      return "E";
    else if (degrees >= 113 && degrees < 158)
      return "SE";
    else if (degrees >= 158 && degrees < 203)
      return "S";
    else if (degrees >= 203 && degrees < 248)
      return "SW";
    else if (degrees >= 248 && degrees < 293)
      return "W";
    else if (degrees >= 293 && degrees < 338)
      return "NW";
    else // covers 338–360 and 0–23
      return "N";
  }

  bool approachHeading(float targetHeading, float thresholdDegrees)
  {
    float currentHeading = offsetDegrees(readHeadingDegrees(), 180);
    float headingError = targetHeading - currentHeading;

    if (abs(headingError) <= thresholdDegrees)
    {
      stopMotor();
      return true;
    }

    float wantedPower = (abs(headingError) * abs(headingError)) / 2000.0;
    float power = std::max(std::min(1.0f, wantedPower), 0.2f);

    if (headingError > 0)
    {
      startMotorRight(power);
    }
    else
    {
      startMotorLeft(power);
    }
    return false;
  }

  void executeProgramState()
  {
    ProgramState nextState = state;
    // Serial.println("Current State: " + String((int)state) + " Previous State: " + String((int)previousState) + " Executed Steps: " + String(executedSteps));

    switch (state)
    {
    case ProgramState::IDLE:
      // Do nothing
      break;
    case ProgramState::LEG:
      if (previousState == ProgramState::IDLE || previousState == ProgramState::TURN)
      {
        Serial.println("Current State: " + String((int)state) + " Previous State: " + String((int)previousState) + " Executed Steps: " + String(executedSteps));
        int distanceCm = executedSteps == 0 ? 20 : executedSteps == 1 ? 13
                                                                      : 20;
        float power = executedSteps == 0 ? 0.75 : executedSteps == 1 ? 0.25
                                                                     : 0.5;
        driveDistance(distanceCm, power);
      }
      if (distanceDriven)
      {
        nextState = executedSteps >= 2 ? ProgramState::IDLE : ProgramState::TURN;
        executedSteps++;
      }

      break;
    case ProgramState::TURN:
      if (previousState == ProgramState::LEG)
      {
        float heading = offsetDegrees(readHeadingDegrees(), 180);
        targetHeading = turnAngleLeft(heading, 110);
      }

      bool doneTurning = approachHeading(targetHeading, 3);
      if (doneTurning)
      {
        nextState = ProgramState::LEG;
      }
      break;
    }

    previousState = state;
    state = nextState;
  }

  void loop()
  {
    float heading = offsetDegrees(readHeadingDegrees(), 180);

    String direction = getHeadingDirection(heading);
    p::lcd::device.printLine(2, "Heading " + direction + " :" + String(heading, 2));

    executeProgramState();

    delay(200);
  }

  void startMotor(float power)
  {

    power = map(power * 100, 0, 100, 0, maxPower);

#define Motor_forward 0
#define Motor_return 1
#define Motor_L_dir_pin 7
#define Motor_R_dir_pin 8
#define Motor_L_pwm_pin 9
#define Motor_R_pwm_pin 10
    analogWrite(Motor_L_pwm_pin, power);
    analogWrite(Motor_R_pwm_pin, power);

    digitalWrite(Motor_R_dir_pin, Motor_forward);
    digitalWrite(Motor_L_dir_pin, Motor_forward);
  }

  void startMotorLeft(float power)
  {

    power = map(power * 100, 0, 100, 0, maxPower);

#define Motor_forward 0
#define Motor_return 1
#define Motor_L_dir_pin 7
#define Motor_R_dir_pin 8
#define Motor_L_pwm_pin 9
#define Motor_R_pwm_pin 10

    analogWrite(Motor_L_pwm_pin, power);
    analogWrite(Motor_R_pwm_pin, power);

    digitalWrite(Motor_R_dir_pin, Motor_forward);
    digitalWrite(Motor_L_dir_pin, Motor_return);
  }

  void startMotorRight(float power)
  {

    power = map(power * 100, 0, 100, 0, maxPower);

#define Motor_forward 0
#define Motor_return 1
#define Motor_L_dir_pin 7
#define Motor_R_dir_pin 8
#define Motor_L_pwm_pin 9
#define Motor_R_pwm_pin 10
    analogWrite(Motor_L_pwm_pin, power);
    analogWrite(Motor_R_pwm_pin, power);

    digitalWrite(Motor_R_dir_pin, Motor_return);
    digitalWrite(Motor_L_dir_pin, Motor_forward);
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
}