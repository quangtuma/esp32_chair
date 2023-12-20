#include "Motor.h"

Motor::Motor()
{

}

void Motor::motorInit(uint8_t pwm, uint8_t dir, uint8_t stop, uint8_t channel, bool isLong)
{
  pwmPin = pwm;
  dirPin = dir;
  stopPin = stop;
  pwmChannel = channel;
  isLongFred = isLong;

  pinMode(dirPin, OUTPUT);
  pinMode(stopPin, INPUT_PULLUP);

  if (isLongFred)
    ledcSetup(pwmChannel, LONG_FRED, RESOLUTION);
  else
    ledcSetup(pwmChannel, SHORT_FRED, RESOLUTION);

  ledcAttachPin(pwmPin, pwmChannel);
}

void Motor::setSpeedMotor(int value)
{
  Serial.printf("Value of %d: %d\n", pwmChannel, value);
  ledcWrite(pwmChannel, value);
}

void Motor::setDirection(int dir)
{
  digitalWrite(dirPin, dir);
}

int Motor::getStopMotor()
{
  return digitalRead(stopPin);
}
