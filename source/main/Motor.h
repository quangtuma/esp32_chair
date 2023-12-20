#ifndef MOTOR_h
#define MOTOR_h

#define RESOLUTION 8
#define LONG_FRED 4000
#define SHORT_FRED 200
#define UP LOW
#define DOWN HIGH

#include <Arduino.h>
#include <ESP.h>

class Motor
{
  private:
    bool isLongFred;
    uint8_t pwmPin;
    uint8_t dirPin;
    uint8_t stopPin;
    uint8_t pwmChannel;
  public:
    Motor();
    void motorInit(uint8_t pwm, uint8_t dirPin, uint8_t stop, uint8_t channel, bool isLong);
    void setSpeedMotor(int value);
    //void setTimerMotor();
    int getStopMotor();
    void setDirection(int dir);
};

#endif