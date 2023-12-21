#include <WiFi.h>
#include <PubSubClient.h>
#include "Motor.h"

WiFiClient espClient;
PubSubClient client(espClient);
const char* ssid = "Cafe_plus";
const char* password = "phamnhuxuong5";
const char* mqtt_server = "broker.emqx.io";
const char* mqtt_username = "emqx"; // username for authentication
const char* mqtt_password = "public";// password for authentication
const int mqtt_port = 1883;
const char* topic_mode = "Mode";
const char* topic_config = "Config";
const char* topic_data = "Data";
const char* topic_control = "Control";

#define MOTOR_NUM 2
long delay_time = 15000;
uint8_t motor_dir_pin[MOTOR_NUM] = { 12, 26 }; //, 32, 2, 17 };
uint8_t motor_pwm_pin[MOTOR_NUM] = { 13, 27 }; //, 33, 15, 16 };
uint8_t motor_stop_pin[MOTOR_NUM] = { 14, 25 }; //, 35, 4, 5 };
uint8_t motor_channel[MOTOR_NUM] = { 0, 1 }; //, 2, 3, 4 };
bool motor_long[MOTOR_NUM] = { true, true }; //, true, false, false };
Motor motorList[MOTOR_NUM];
uint8_t duty = 200;

enum EnumMode{
  Auto = 0,
  Manual,
  Getsure
};

EnumMode MODE = Auto;

// motor region
void setupMotors(){
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

void runAuto(){
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

void setupMqtt(){
  //connecting to a mqtt broker
  client.setServer(mqtt_broker, mqtt_port);
  client.setCallback(callback);
  while (!client.connected()) {
      String client_id = "esp32-client-";
      client_id += String(WiFi.macAddress());
      Serial.printf("The client %s connects to the public mqtt broker\n", client_id.c_str());
      if (client.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
          Serial.println("Public emqx mqtt broker connected");
      } else {
          Serial.print("failed with state ");
          Serial.print(client.state());
          delay(1000);
      }
  }
  // publish and subscribe
  client.publish(topic, "Hi EMQX I'm ESP32 ^^");
  client.subscribe(topic);
}

void setMode(char* mode){
  switch(mode)
  {
    case '0': 
      MODE = Auto;
      break;
    case '1':
      MODE = Manual;
      break;
    case '2': 
      MODE = Getsure;
      break;
  }
}

void setConfig(){

}

uint8_t timeConfigArray[MOTOR_NUM];

void setControl(char* config){
  unsigned int startIndex, nextIndex, index_array;
  String configStr = String(config);
  String configArray[MOTOR_NUM];
  while(startIndex < configStr.length)
  {
    nextIndex = config.indexOf('-', startIndex);
    configArray[index_array] = config.substring(startIndex, nextIndex);
    index_array++;
    startIndex = nextIndex + 1;
  }
  for(uint8_t index = 0; index < MOTOR_NUM, index++)
  {
    timeConfigArray[index] = configArray[index].toInt();
  }
}

void callback(char *topic, byte *payload, unsigned int length) {
  Serial.printf("Message arrived in topic: %s", );
  Serial.println(topic);
  char* message;
  Serial.print("Message:");
  for (int i = 0; i < length; i++) {
    message += (char) payload[i]);
    Serial.print((char) payload[i]);
  }

  switch (topic)
  {
    case topic_mode:
      setMode(message);
      break;
    case topic_config:
      setConfig();
      break;
    case topic_control:
      setControl();
      break;
    default:
      break;
  }
  Serial.println();
  Serial.println("-----------------------");
}


void setup(){
  Serial.begin(115200);
  setupMotors();
}

void loop(){

  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  runAuto();
}

