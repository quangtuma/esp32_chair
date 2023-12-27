#ifndef MOTOR_h
#define MOTOR_h

#define RESOLUTION 8
#define STEP_FRED 4000
#define DC_FRED 30000
#define UP HIGH
#define DOWN LOW

#include <Arduino.h>
#include <ESP.h>

class Motor
{
  private:
    uint8_t pwmPin;
    uint8_t dirPin;
    uint8_t stopPin;
    uint8_t pwmChannel;
  public:
    Motor();
    void motorInit(uint8_t pwm, uint8_t dirPin, uint8_t stop, uint8_t channel);
    void setSpeedMotor(int value);
    //void setTimerMotor();
    int getStopMotor();
    void setDirection(int dir);
};

class DCMotor
{
  private:
    uint8_t pwm1Pin;
    uint8_t pwm2Pin;
    uint8_t pwmChannel1;
    uint8_t pwmChannel2;
    uint8_t stopPin;
  public:
    DCMotor();
    void dcMotorInit(uint8_t pwm1, uint8_t pwm2, uint8_t stop, uint8_t channel1, uint8_t channel2);
    void setStopMotor();
    int getStopMotor();
    void setSpeedUp(uint8_t duty);
    void setSpeedDown(uint8_t duty);
};

#endif