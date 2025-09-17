#include <LiquidCrystal.h>
#include <Arduino.h>
#include "exercises.h"

namespace ex_w3_3_4
{
#define Motor_forward 0
#define Motor_return 1
#define Motor_L_dir_pin 7
#define Motor_R_dir_pin 8
#define Motor_L_pwm_pin 9
#define Motor_R_pwm_pin 10

  void setup()
  {
    Serial.begin(9600);
  }

  void loop()
  {
    int pwm_R = 1000;
    int pwm_L = 1000;

    analogWrite(Motor_L_pwm_pin, pwm_L);
    analogWrite(Motor_R_pwm_pin, pwm_R);

    for (int i = 99; i < 100; i--)
    {

      ////// Direction
      digitalWrite(Motor_R_dir_pin, Motor_return);
      digitalWrite(Motor_L_dir_pin, Motor_return);
      delay(2000);
      digitalWrite(Motor_R_dir_pin, Motor_forward);
      digitalWrite(Motor_L_dir_pin, Motor_forward);
      delay(2000);
    }
  }
}