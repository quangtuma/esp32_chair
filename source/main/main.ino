#include "Motor.h"
#define MOTOR_NUM 2
long delay_time = 15000;

uint8_t motor_dir_pin[MOTOR_NUM] = { 12, 26 }; //, 32, 2, 17 };
uint8_t motor_pwm_pin[MOTOR_NUM] = { 13, 27 }; //, 33, 15, 16 };
uint8_t motor_stop_pin[MOTOR_NUM] = { 14, 25 }; //, 35, 4, 5 };
uint8_t motor_channel[MOTOR_NUM] = { 0, 1 }; //, 2, 3, 4 };
bool motor_long[MOTOR_NUM] = { true, true }; //, true, false, false };
Motor motorList[MOTOR_NUM];

uint8_t duty = 200;

void setupMotors()
{
  // to init motors
  for (int index = 0; index < MOTOR_NUM; index++)
  {
    motorList[index].motorInit(motor_pwm_pin[index], motor_dir_pin[index], motor_stop_pin[index], motor_channel[index], motor_long[index]);
  }

  // to run initial
  for (uint8_t index = 0; index < MOTOR_NUM; index++)
  {
    motorList[index].setDirection(UP);
    motorList[index].setSpeedMotor(duty);
  }
  delay(1500);
  for (uint8_t index = 0; index < MOTOR_NUM; index++)
  {
    motorList[index].setDirection(DOWN);
    motorList[index].setSpeedMotor(duty);
  }
  // to stop point
  while(motorList[0].getStopMotor() == HIGH 
        || motorList[1].getStopMotor() == HIGH
        // || motorList[2].getStopMotor() == HIGH
        // || motorList[3].getStopMotor() == HIGH
        // || motorList[4].getStopMotor() == HIGH
  )
  {
    Serial.printf("In while!\n");
    if (motorList[0].getStopMotor() == LOW) 
      motorList[0].setSpeedMotor(0);
    if (motorList[1].getStopMotor() == LOW) 
      motorList[1].setSpeedMotor(0);
    // if (motorList[2].getStopMotor() == LOW) 
    //   motorList[2].setSpeedMotor(0);
    // if (motorList[3].getStopMotor() == LOW) 
    //   motorList[3].setSpeedMotor(0);
    // if (motorList[4].getStopMotor() == LOW) 
    //   motorList[4].setSpeedMotor(0);
  }
  Serial.printf("Turn root!\n");
  delay(1000);

  // to init postion 
  motorList[0].setDirection(UP);
  motorList[0].setSpeedMotor(duty);
  // motorList[2].setDirection(1);
  // motorList[2].setSpeedMotor(duty);
  delay(delay_time);
  motorList[0].setSpeedMotor(0);
  //motorList[2].setSpeedMotor(0);
  Serial.printf("Already run!\n");
  delay(2000);
}

void setup(){
  Serial.begin(115200);
  setupMotors();
}

void runAuto()
{
  // run up
  motorList[0].setDirection(DOWN);
  motorList[0].setSpeedMotor(duty);
  motorList[1].setDirection(UP);
  motorList[1].setSpeedMotor(duty);
  // motorList[2].setDirection(0);
  // motorList[2].setSpeedMotor(duty);
  // motorList[3].setDirection(1);
  // motorList[3].setSpeedMotor(duty);
  // motorList[4].setDirection(0);
  // motorList[4].setSpeedMotor(duty);
  delay(delay_time);

  // run down
  motorList[0].setDirection(UP);
  motorList[0].setSpeedMotor(duty);
  motorList[1].setDirection(DOWN);
  motorList[1].setSpeedMotor(duty);
  // motorList[2].setDirection(0);
  // motorList[2].setSpeedMotor(duty);
  // motorList[3].setDirection(1);
  // motorList[3].setSpeedMotor(duty);
  // motorList[4].setDirection(0);
  // motorList[4].setSpeedMotor(duty);
  delay(delay_time);
}
 
void loop(){
  runAuto();
}