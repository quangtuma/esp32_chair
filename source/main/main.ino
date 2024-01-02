#include <WiFi.h>
#include <PubSubClient.h>
#include "Motor.h"
#include <string.h>

WiFiClient espClient;
PubSubClient client(espClient);
const uint8_t LED_MQTT_PIN = 13;
const char* ssid = "Phi Long";
const char* password = "98765432";
const char* mqtt_server = "public.mqtthq.com";
const int mqtt_port = 1883;
const char* topic_mode = "Mode";
char* topic_config = "Config";
char* topic_data = "Data";
char* topic_control_gen = "Control";
#define MOTOR_NUM 5
char* topic_control[MOTOR_NUM] = { "Control/LeftLeg", "Control/LeftFoot", "Control/Hands", "Control/RightLeg", "Control/RightFoot" };

#define STEP_MOTOR_NUM 2
long delay_time_step = 5000;
uint8_t step_motor_pwm_pin[STEP_MOTOR_NUM] = { 27, 25 };
uint8_t step_motor_dir_pin[STEP_MOTOR_NUM] = { 26, 33 };
uint8_t step_motor_stop_pin[STEP_MOTOR_NUM] = { 32, 35 };
uint8_t step_motor_channel[STEP_MOTOR_NUM] = { 0, 1 };
Motor stepMotorList[STEP_MOTOR_NUM];
uint8_t duty_step = 250;

#define DC_MOTOR_NUM 3
long delay_time_dc = 3000;
uint8_t dc_motor_pwm1_pin[DC_MOTOR_NUM] = { 2, 16, 18 };
uint8_t dc_motor_pwm2_pin[DC_MOTOR_NUM] = { 4, 17, 19 };
uint8_t dc_motor_stop_pin[DC_MOTOR_NUM] = { 34, 39, 36 };
uint8_t dc_motor_channel1[DC_MOTOR_NUM] = { 2, 4, 6 };
uint8_t dc_motor_channel2[DC_MOTOR_NUM] = { 3, 5, 7 };
DCMotor dcMotorList[DC_MOTOR_NUM];
uint8_t duty_dc = 200;

int Auto_Number = 1;
bool Auto_Restart = true; 

enum EnumMode{
  Auto = 0,
  Manual,
  Gesture
};

EnumMode MODE = Auto;

// motor region
void setupMotors(){
  // to init motors
  for (uint8_t index = 0; index < STEP_MOTOR_NUM; index++)
  {
    stepMotorList[index].motorInit(step_motor_pwm_pin[index], step_motor_dir_pin[index], step_motor_stop_pin[index], step_motor_channel[index]);
  }

  delay(100);

  for (int index = 0; index < DC_MOTOR_NUM; index++)
  {
    dcMotorList[index].dcMotorInit(dc_motor_pwm1_pin[index], dc_motor_pwm2_pin[index], dc_motor_stop_pin[index], dc_motor_channel1[index], dc_motor_channel2[index]);
  }

  // to run initial
  for (uint8_t index = 0; index < STEP_MOTOR_NUM; index++)
  {
    stepMotorList[index].setDirection(UP);
    stepMotorList[index].setSpeedMotor(duty_step);
  }

  for (uint8_t index = 0; index < DC_MOTOR_NUM - 1; index++)
  {
    dcMotorList[index].setSpeedDown(duty_dc);
  }

  delay(1000);

  for (uint8_t index = 0; index < STEP_MOTOR_NUM; index++)
  {
    stepMotorList[index].setDirection(DOWN);
    stepMotorList[index].setSpeedMotor(duty_step);
  }

  for (uint8_t index = 0; index < DC_MOTOR_NUM - 1; index++)
  {
    dcMotorList[index].setSpeedUp(duty_dc);
  }

  // to stop point
  while(stepMotorList[0].getStopMotor() == HIGH
        || stepMotorList[1].getStopMotor() == HIGH
        || dcMotorList[0].getStopMotor() == HIGH
        || dcMotorList[1].getStopMotor() == HIGH
  )
  {
    Serial.printf("In while!\n");
    if (stepMotorList[0].getStopMotor() == LOW)
      stepMotorList[0].setSpeedMotor(0);
    if (stepMotorList[1].getStopMotor() == LOW)
      stepMotorList[1].setSpeedMotor(0);
    if (dcMotorList[0].getStopMotor() == LOW)
      dcMotorList[0].setStopMotor();
    if (dcMotorList[1].getStopMotor() == LOW)
      dcMotorList[1].setStopMotor();
  }
  Serial.printf("Turn root!\n");
  delay(1000);
  
  stepMotorList[1].setDirection(UP);
  stepMotorList[1].setSpeedMotor(duty_step);
  delay(delay_time_step);
  
  stepMotorList[1].setSpeedMotor(0);
}

// unsigned long lastMillis_1 = delay_time_step, lastMillis_2 = 0, lastMillis_3 = 0, initTime = 0;
void runAuto(){
  //Serial.println("Run auto!");
  // run up
  /* use milli()
  unsigned long currentMillis = millis();
  if (currentMillis <= delay_time_step)
  {
    return;
  }
  if (currentMillis - lastMillis_1 >= delay_time_step * 2){
    lastMillis_1 = currentMillis;
    
    Serial.printf("L1 - L1 time: %d\n", lastMillis_1);
    Serial.printf("L1 - L2 time: %d\n\n", lastMillis_2);
    Serial.printf("Run up!\n");
    stepMotorList[0].setDirection(DOWN);
    stepMotorList[0].setSpeedMotor(duty);
    stepMotorList[1].setDirection(UP);
    stepMotorList[1].setSpeedMotor(duty);
  }

  // run down
  if (currentMillis - lastMillis_2 >= delay_time_step * 2){
    lastMillis_2 = currentMillis;
    Serial.printf("L2 - L1 time: %d\n", lastMillis_1);
    Serial.printf("L2 - L2 time: %d\n", lastMillis_2);
    Serial.printf("Run down!\n\n");
    stepMotorList[0].setDirection(UP);
    stepMotorList[0].setSpeedMotor(duty);
    stepMotorList[1].setDirection(DOWN);
    stepMotorList[1].setSpeedMotor(duty);
  }
  */
  for (uint8_t index = 0; index < 3; index++)
  {
    dcMotorList[0].setSpeedDown(duty_dc);
    delay(delay_time_dc);
    dcMotorList[0].setSpeedUp(duty_dc);
    delay(delay_time_dc);
  }
  dcMotorList[0].setStopMotor();
  delay(500);

  stepMotorList[0].setDirection(UP);
  stepMotorList[0].setSpeedMotor(duty_step);
  stepMotorList[1].setDirection(DOWN);
  stepMotorList[1].setSpeedMotor(duty_step);
  delay(delay_time_step);
  
  stepMotorList[0].setSpeedMotor(0);
  stepMotorList[1].setSpeedMotor(0);
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

  stepMotorList[0].setDirection(DOWN);
  stepMotorList[0].setSpeedMotor(duty_step);
  stepMotorList[1].setDirection(UP);
  stepMotorList[1].setSpeedMotor(duty_step);
  delay(delay_time_step);

  stepMotorList[0].setSpeedMotor(0);
  stepMotorList[1].setSpeedMotor(0);
  delay(500);
}

// mqtt region
void setupWifi(){
  // connecting to a WiFi network
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to the WiFi network");
}

void callback(char *topic, byte *payload, unsigned int length) {
  Serial.print("Message arrived in topic: ");
  Serial.println(topic);
  String message;
  Serial.print("Message: ");
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
    Serial.print((char) payload[i]);
  }
  Serial.println();

  String topicStr = String(topic);

  if (topicStr == topic_mode){
    if(message == "0")
    {
      Serial.println("Set Mode: Auto");
      MODE = Auto;
      Auto_Restart = true;
    }
    else if (message == "1")
    {
      Serial.println("Set Mode: Manual");
      MODE = Manual;
    }
    else if (message == "2")
    {
      Serial.println("Set Mode: Gesture");
      MODE = Gesture;
    }
  }
  else if(topicStr.startsWith(topic_control_gen)){
    if (MODE == Manual)
      if (topicStr == topic_control[0]){
        if(message == "0")
          stepMotorList[0].setSpeedMotor(0);
        else if(message == "1")
        {
          Serial.println("Chan trai len");
          stepMotorList[0].setDirection(UP);
          stepMotorList[0].setSpeedMotor(duty_step);
        }
        else if(message == "2")
        {
          Serial.println("Chan trai xuong");
          stepMotorList[0].setDirection(DOWN);
          stepMotorList[0].setSpeedMotor(duty_step);
        }
      }
      else if (topicStr == topic_control[1]){
        if(message == "0")
          dcMotorList[0].setStopMotor();
        else if(message == "1")
          dcMotorList[0].setSpeedUp(duty_dc);
        else if(message == "2")
          dcMotorList[0].setSpeedDown(duty_dc);
      }
      else if (topicStr == topic_control[2]){
        if(message == "0")
          dcMotorList[2].setStopMotor();
        else if(message == "1")
          dcMotorList[2].setSpeedUp(duty_dc);
        else if(message == "2")
          dcMotorList[2].setSpeedDown(duty_dc);
      }
      else if (topicStr == topic_control[3]){
        if(message == "0")
          stepMotorList[1].setSpeedMotor(0);
        else if(message == "1")
        {
          Serial.println("Chan phai len");
          stepMotorList[1].setDirection(UP);
          stepMotorList[1].setSpeedMotor(duty_step);
        }
        else if(message == "2")
        {
          Serial.println("Chan phai xuong");
          stepMotorList[1].setDirection(DOWN);
          stepMotorList[1].setSpeedMotor(duty_step);
        }
      }
      else if (topicStr == topic_control[4]){
        if(message == "0")
          dcMotorList[1].setStopMotor();
        else if(message == "1")
          dcMotorList[1].setSpeedUp(duty_dc);
        else if(message == "2")
          dcMotorList[1].setSpeedDown(duty_dc);
      }
  }
  else if(topicStr.startsWith("Auto")){
    if(topicStr.endsWith("Number")){
      Auto_Number = message.toInt();
    }
    else if(topicStr.endsWith("Restart")){
      Auto_Restart = true;
    }
  }
}

void reconnect(){
  //connecting to a mqtt broker
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  while (!client.connected()) {
      String client_id = "Chair-client";
      Serial.printf("The client %s connects to the public mqtt broker\n", client_id.c_str());
      if (client.connect(client_id.c_str())) {
        Serial.println("Public MQTTHQ broker connected");
        digitalWrite(LED_MQTT_PIN, HIGH);
      } else {
        Serial.print("failed with state ");
        Serial.println(client.state());
        digitalWrite(LED_MQTT_PIN, LOW);
        delay(500);
      }
  }
  // publish and subscribe
  client.publish("Initial", "Hi MQTTHQ I'm ESP32");
  client.subscribe("Initial");
  client.subscribe(topic_mode);
  client.subscribe(topic_data);
  client.subscribe("Auto/Number");
  client.subscribe("Auto/Restart");
  for(uint8_t index = 0; index < MOTOR_NUM; index++){
    client.subscribe(topic_control[index]);
  }
}

void runManual(){
  //Serial.println("Run manual!");
  if (stepMotorList[0].getStopMotor() == LOW)
    stepMotorList[0].setSpeedMotor(0);
  if (stepMotorList[1].getStopMotor() == LOW)
    stepMotorList[1].setSpeedMotor(0);
  if (dcMotorList[0].getStopMotor() == LOW)
    dcMotorList[0].setStopMotor();
  if (dcMotorList[1].getStopMotor() == LOW)
    dcMotorList[1].setStopMotor();
}

void runGesture(){
  Serial.println("Run gesture!");
}

void setup(){
  Serial.begin(115200);
  pinMode(LED_MQTT_PIN, OUTPUT);
  setupMotors();
  setupWifi();
  // initTime = millis();
  // Serial.printf("Init time: %d\n", initTime);
  //while (millis() < 5000);
  // lastMillis_1 = initTime + delay_time_step;
  // Serial.printf("L1 time: %d\n", lastMillis_1);
  // lastMillis_2 = initTime;
}


void loop(){

  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  switch(MODE)
  {
    case Auto:
      if(Auto_Restart){
        client.publish("Auto/IsRun", "1");
        for(uint8_t index = 0; index < Auto_Number; index++){
          runAuto();
          Serial.println("OUT AUTO!");
        }
        Auto_Restart = false;
        client.publish("Auto/IsRun", "0");
      }
      break;
    case Manual:
      runManual();
      break;
    case Gesture:
      runGesture();
      break;
    default:
      break;
  }
}
