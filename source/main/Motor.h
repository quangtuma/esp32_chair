#ifndef MOTOR_h
#define MOTOR_h

#define RESOLUTION 8
#define LONG_FRED 4000
#define SHORT_FRED 200

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
    Motor(uint8_t *pwm, uint8_t *dirPin, uint8_t *stop, uint8_t *channel, bool *isLong);
    void motorInit();
    void setSpeedMotor(int value);
    //void setTimerMotor();
    int getStopMotor();
    void setDirection(int dir);
};

#endif