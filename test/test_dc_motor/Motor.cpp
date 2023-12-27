#include "Motor.h"

Motor::Motor()
{

}

void Motor::motorInit(uint8_t pwm, uint8_t dir, uint8_t stop, uint8_t channel)
{
  pwmPin = pwm;
  dirPin = dir;
  stopPin = stop;
  pwmChannel = channel;

  pinMode(dirPin, OUTPUT);
  pinMode(pwmPin, OUTPUT);
  pinMode(stopPin, INPUT_PULLUP);

  ledcSetup(pwmChannel, STEP_FRED, RESOLUTION);
  ledcAttachPin(pwmPin, pwmChannel);
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

// DC Motor
DCMotor::DCMotor(){

}
void DCMotor::dcMotorInit(uint8_t pwm1, uint8_t pwm2, uint8_t stop, uint8_t channel1, uint8_t channel2){
  pwm1Pin = pwm1;
  pwm2Pin = pwm2;
  pwmChannel1 = channel1;
  pwmChannel2 = channel2;
  stopPin = stop;

  pinMode(pwm1Pin, OUTPUT);
  pinMode(pwm2Pin, OUTPUT);
  pinMode(stopPin, INPUT_PULLUP);

  ledcSetup(pwmChannel1, DC_FRED, RESOLUTION);
  ledcSetup(pwmChannel2, DC_FRED, RESOLUTION);

  ledcAttachPin(pwm1Pin, pwmChannel1);
  ledcAttachPin(pwm2Pin, pwmChannel2);
}
void DCMotor::setStopMotor(){
  ledcWrite(pwmChannel1, 0);
  ledcWrite(pwmChannel2, 0);
}
int DCMotor::getStopMotor(){
  return digitalRead(stopPin);
}
void DCMotor::setSpeedUp(uint8_t duty){
  ledcWrite(pwmChannel1, duty);
  ledcWrite(pwmChannel2, 0);
}
void DCMotor::setSpeedDown(uint8_t duty){
  ledcWrite(pwmChannel1, 0);
  ledcWrite(pwmChannel2, duty);
}
