#include <WiFi.h>
#include <PubSubClient.h>
#include "Motor.h"

#define DC_MOTOR_NUM 3
long delay_time_dc = 3000;
uint8_t dc_motor_pwm1_pin[DC_MOTOR_NUM] = { 2, 16, 18 };
uint8_t dc_motor_pwm2_pin[DC_MOTOR_NUM] = { 4, 17, 19 };
uint8_t dc_motor_stop_pin[DC_MOTOR_NUM] = { 34, 39, 36 };
uint8_t dc_motor_channel1[DC_MOTOR_NUM] = { 2, 4, 6 };
uint8_t dc_motor_channel2[DC_MOTOR_NUM] = { 3, 5, 7 };
DCMotor dcMotorList[DC_MOTOR_NUM];
uint8_t duty_dc = 200;

void setup(){
  Serial.begin(115200);

  for (int index = 0; index < DC_MOTOR_NUM; index++)
  {
    dcMotorList[index].dcMotorInit(dc_motor_pwm1_pin[index], dc_motor_pwm2_pin[index], dc_motor_stop_pin[index], dc_motor_channel1[index], dc_motor_channel2[index]);
  }

  for (uint8_t index = 0; index < DC_MOTOR_NUM - 1; index++)
  {
    dcMotorList[index].setSpeedDown(duty_dc);
  }

  delay(1000);

  for (uint8_t index = 0; index < DC_MOTOR_NUM - 1; index++)
  {
    dcMotorList[index].setSpeedUp(duty_dc);
  }

  // to stop point
  while(dcMotorList[0].getStopMotor() == HIGH
        || dcMotorList[1].getStopMotor() == HIGH
  )
  {
    Serial.printf("In while!\n");
    if (dcMotorList[0].getStopMotor() == LOW)
      dcMotorList[0].setStopMotor();
    if (dcMotorList[1].getStopMotor() == LOW)
      dcMotorList[1].setStopMotor();
  }
  Serial.printf("Turn root!\n");
  delay(1000);
}

void loop(){
  for (uint8_t index = 0; index < 3; index++)
  {
    dcMotorList[0].setSpeedDown(duty_dc);
    delay(delay_time_dc);
    dcMotorList[0].setSpeedUp(duty_dc);
    delay(delay_time_dc);
  }
  dcMotorList[0].setStopMotor();
  delay(500);

  for (uint8_t index = 0; index < 3; index++)
  {
    dcMotorList[1].setSpeedDown(duty_dc);
    delay(delay_time_dc);
    dcMotorList[1].setSpeedUp(duty_dc);
    delay(delay_time_dc);
  }
  dcMotorList[1].setStopMotor();
  delay(500);
}