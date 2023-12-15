#include "Motor.h"

Motor::Motor(uint8_t *pwm, uint8_t *dir, uint8_t *stop, uint8_t *channel, bool *isLong)
{
  &pwmPin = pwm;
  &dirPin = dir;
  &stopPin = stop;
  &pwmChannel = channel;
  &isLongFred = isLong;
}

void Motor::motorInit()
{
  pinMode(dirPin, OUTPUT);
  pinMode(stopPin, INPUT_PULLUP);

  if (isLongFred)
    ledcSetup(pwmChannel, LONG_FRED, RESOLUTION);
  else
    ledcSetup(pwmChannel, SHORT_FRED, RESOLUTION);

  ledcAttachPin(pwmPin, pwmChannel);
  digitalWrite(dirPin, LOW);
}

void Motor::setSpeedMotor(int value)
{
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
