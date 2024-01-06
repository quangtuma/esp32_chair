#include <WiFi.h>
#include <PubSubClient.h>
#include "Motor.h"
#include <string.h>
#include <Wire.h>
#include "MAX30100_PulseOximeter.h"
#include <LiquidCrystal_I2C.h>
//#include <MPU6050_tockn.h>
#include "SinricPro.h"
#include "SinricProSwitch.h"
#include <MPU6050.h>
#include <SoftwareSerial.h>

#define REPORTING_PERIOD_MS 1000

SoftwareSerial SoftSerial(2, 15);//D2 = RX -- D3 = TX

WiFiClient espClient;
PubSubClient client(espClient);
const uint8_t LED_MQTT_PIN = 12;
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

#define CHECK_LIMIT_
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
uint8_t dc_motor_pwm1_pin[DC_MOTOR_NUM] = { 23, 16, 18 };
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
EnumMode MODE = Manual;

PulseOximeter pox;
uint32_t tsLastReport = 0;
LiquidCrystal_I2C lcd(0x27, 20, 4);

// MPU6050 mpu6050(Wire);
// float angleCurrent = 0;
MPU6050 mpu;
float angleCurrent;
int16_t ax, ay, az, gx, gy, gz;
unsigned long mpuLastTime = 0;

#define APP_KEY           "a07486ba-e712-42cf-b47f-29405a1f4011"      // Should look like "de0bxxxx-1x3x-4x3x-ax2x-5dabxxxxxxxx"
#define APP_SECRET        "719e0c0e-c611-4492-a77e-2c3429ab5ca8-930fb95e-54de-475e-aab1-bd7d1f689099"   // Should look like "5f36xxxx-x3x7-4x3x-xexe-e86724a9xxxx-4c4axxxx-3x3x-x5xe-x9x3-333d65xxxxxx"
#define SWITCH_ID         "657bd06f2e988977c418631e"    // Should look like "5dc1564130xxxxxxxxxxxxxx"

bool myPowerState = false;

void initMotor(){
  // to run initial
  for (uint8_t index = 0; index < STEP_MOTOR_NUM; index++)
  {
    stepMotorList[index].setDirection(UP);
    stepMotorList[index].setSpeedMotor(duty_step);
  }

  for (uint8_t index = 0; index < DC_MOTOR_NUM - 1; index++)
    dcMotorList[index].setSpeedDown(duty_dc);

  delay(1000);

  for (uint8_t index = 0; index < STEP_MOTOR_NUM; index++)
  {
    stepMotorList[index].setDirection(DOWN);
    stepMotorList[index].setSpeedMotor(duty_step);
  }

  for (uint8_t index = 0; index < DC_MOTOR_NUM - 1; index++)
    dcMotorList[index].setSpeedUp(duty_dc);

  delay(100);

  // to stop point
  
  Serial.print("In while");
  while(stepMotorList[0].getStopMotor() == HIGH
        || stepMotorList[1].getStopMotor() == HIGH
        || dcMotorList[0].getStopMotor() == HIGH
        || dcMotorList[1].getStopMotor() == HIGH
  )
  {
    Serial.printf(".");
    if (stepMotorList[0].getStopMotor() == LOW)
      stepMotorList[0].setSpeedMotor(0);
    if (stepMotorList[1].getStopMotor() == LOW)
      stepMotorList[1].setSpeedMotor(0);
    if (dcMotorList[0].getStopMotor() == LOW)
      dcMotorList[0].setStopMotor();
    if (dcMotorList[1].getStopMotor() == LOW)
      dcMotorList[1].setStopMotor();
      
    delay(200);
  }

  stepMotorList[1].setDirection(UP);
  stepMotorList[1].setSpeedMotor(duty_step);
  delay(delay_time_step);    

  stepMotorList[1].setSpeedMotor(0);
  
  Serial.printf("\nMotor Ready!\n");
  delay(1000);
}

// motor region
void setupMotors(){
  // to init motors
  for (uint8_t index = 0; index < STEP_MOTOR_NUM; index++)
    stepMotorList[index].motorInit(step_motor_pwm_pin[index], 
                                    step_motor_dir_pin[index], step_motor_stop_pin[index], step_motor_channel[index]);

  delay(100);

  for (int index = 0; index < DC_MOTOR_NUM; index++)
    dcMotorList[index].dcMotorInit(dc_motor_pwm1_pin[index], 
                                    dc_motor_pwm2_pin[index], dc_motor_stop_pin[index], dc_motor_channel1[index], dc_motor_channel2[index]);

#ifdef CHECK_LIMIT
  initMotor();
#endif
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
      SoftSerial.println("Command: 0");
    }
    else if (message == "1")
    {
      Serial.println("Set Mode: Manual");
      MODE = Manual;
      stepMotorList[0].setSpeedMotor(0);
      stepMotorList[1].setSpeedMotor(0);
      dcMotorList[0].setStopMotor();
      dcMotorList[1].setStopMotor();
      dcMotorList[2].setStopMotor();
      SoftSerial.println("Command: 0");
    }
    else if (message == "2")
    {
      Serial.println("Set Mode: Gesture");
      MODE = Gesture;
      SoftSerial.println("Command: 1");
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
          if (stepMotorList[0].getStopMotor() == LOW)
            stepMotorList[0].setSpeedMotor(0);
          else {
            Serial.println("Chan trai xuong");
            stepMotorList[0].setDirection(DOWN);
            stepMotorList[0].setSpeedMotor(duty_step);
          }
        }
      }
      else if (topicStr == topic_control[1]){
        if(message == "0")
          dcMotorList[0].setStopMotor();
        else if(message == "1"){
          if (dcMotorList[0].getStopMotor() == HIGH)
            dcMotorList[0].setSpeedUp(duty_dc);
        }
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
          if (stepMotorList[1].getStopMotor() == LOW)
            stepMotorList[1].setSpeedMotor(0);
          else {
            Serial.println("Chan phai xuong");
            stepMotorList[1].setDirection(DOWN);
            stepMotorList[1].setSpeedMotor(duty_step);
          }
        }
      }
      else if (topicStr == topic_control[4]){
        if(message == "0")
          dcMotorList[1].setStopMotor();
        else if(message == "1"){
          if (dcMotorList[1].getStopMotor() == HIGH)
            dcMotorList[1].setSpeedUp(duty_dc);
        }
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

void reconnectMqtt(){
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

bool onPowerState(const String &deviceId, bool &state) {
  Serial.printf("Device %s turned %s \r\n", deviceId.c_str(), state?"on auto":"off auto");
  myPowerState = state;
  if (state){
    MODE = Auto;
    Auto_Restart = state;
  }
  else 
    MODE = Manual;
  return true; // request handled properly
}

// setup function for SinricPro
void setupSinricPro() {
  // add device to SinricPro
  SinricProSwitch& mySwitch = SinricPro[SWITCH_ID];

  // set callback function to device
  mySwitch.onPowerState(onPowerState);

  // setup SinricPro
  SinricPro.onConnected([](){ 
    Serial.printf("Connected to SinricPro\r\n"); 
    if (SinricPro.isConnected())
    {
      SinricProSwitch& mySwitch = SinricPro[SWITCH_ID];
      if(!mySwitch.sendPowerStateEvent(myPowerState)) {
        Serial.printf("Something went wrong...could not send Event to server!\r\n");
      }
  }});

  SinricPro.onDisconnected([](){ Serial.printf("Disconnected from SinricPro\r\n"); });
  //SinricPro.restoreDeviceStates(true); // Uncomment to restore the last known state from the server.
  SinricPro.begin(APP_KEY, APP_SECRET);
}

void runAuto(){
  for (uint8_t index = 0; index < 3; index++)
  {
    dcMotorList[0].setSpeedDown(duty_dc);
    dcMotorList[2].setSpeedDown(duty_dc);
    delay(delay_time_dc);
    dcMotorList[0].setSpeedUp(duty_dc);
    dcMotorList[2].setSpeedUp(duty_dc);
    delay(delay_time_dc);
  }
  dcMotorList[0].setStopMotor();
  dcMotorList[2].setStopMotor();
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
    dcMotorList[2].setSpeedDown(duty_dc);
    delay(delay_time_dc);
    dcMotorList[1].setSpeedUp(duty_dc);
    dcMotorList[2].setSpeedUp(duty_dc);
    delay(delay_time_dc);
  }
  dcMotorList[1].setStopMotor();
  dcMotorList[2].setStopMotor();
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

void runManual(){
  //Serial.println("Run manual!");
}

void runGesture(){
  if(millis() - mpuLastTime > REPORTING_PERIOD_MS){
    Serial.println("Run gesture!");
    mpuLastTime = millis();
  }
}

void setup(){
  Serial.begin(115200);
  pinMode(2,INPUT);
  pinMode(15,OUTPUT);
  SoftSerial.begin(115200);
  pinMode(LED_MQTT_PIN, OUTPUT);
  setupMotors();
  setupWifi();
  setupSinricPro();
}

void processSerialMessage(String message){
  if (message.startsWith("BPM:")){
    Serial.println("Message: " + message);
    uint8_t startIndex = message.indexOf(':') + 1;
    uint8_t lastIndex = message.indexOf(',');
    String bpm = message.substring(startIndex, lastIndex);
    if (bpm.equals(""))
      bpm = "00";

    startIndex = message.indexOf(':', lastIndex) + 1;
    lastIndex = message.indexOf('\n') - 1;
    String spo2 = message.substring(startIndex, lastIndex);
    if (spo2.equals(""))
      spo2 = "00";

    String dataStr = bpm + "," + spo2;
    Serial.printf("Message Push: %s\n", dataStr);
    client.publish(topic_data, dataStr.c_str());
  }
  else if (message.startsWith("Angle:") && MODE == Gesture){
    Serial.println("Message: " + message);
    uint8_t startIndex = message.indexOf(':') + 1;
    uint8_t lastIndex = message.indexOf('\n') - 1;
    float angle = message.substring(startIndex, lastIndex).toFloat();
    
    if (angle >= angleCurrent + 2){
      if (stepMotorList[0].getStopMotor() == LOW || stepMotorList[1].getStopMotor() == LOW){
        for(uint8_t id = 0; id < STEP_MOTOR_NUM; id++){
          stepMotorList[id].setSpeedMotor(0);
        }
      }
      else{
        Serial.println("RUN UP!");
        for(uint8_t id = 0; id < STEP_MOTOR_NUM; id++){
          stepMotorList[id].setDirection(UP);
          stepMotorList[id].setSpeedMotor(duty_step);
        }
      }
    }
    else if (angle <= angleCurrent - 2){
      Serial.println("RUN DOWN!");
      for(uint8_t id = 0; id < STEP_MOTOR_NUM; id++){
        stepMotorList[id].setDirection(DOWN);
        stepMotorList[id].setSpeedMotor(duty_step);
      }
    }
    else {
      Serial.println("RUN STOP!");
      for(uint8_t id = 0; id < STEP_MOTOR_NUM; id++){
        stepMotorList[id].setSpeedMotor(0);
      }
    }

    angleCurrent = angle;
  }
}

void loop(){

  if (!client.connected()) {
    reconnectMqtt();
  }
  client.loop();
  SinricPro.handle();

  String message = "";
  while(SoftSerial.available()){
    char ch = (char)SoftSerial.read();
    message += ch;
    if (ch == 10)
      break;
  }

  if (!message.equals(""))
    processSerialMessage(message);

  switch(MODE)
  {
    case Auto:
      if(Auto_Restart){
        if (!client.connected())
          reconnectMqtt();
        client.publish("Auto/IsRun", "1");

        initMotor();
        for(uint8_t index = 0; index < Auto_Number; index++){
          runAuto();
          Serial.println("OUT AUTO!");
        }
        Auto_Restart = false;

        if (!client.connected())
          reconnectMqtt();
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
