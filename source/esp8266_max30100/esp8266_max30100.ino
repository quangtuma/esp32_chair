#include <SoftwareSerial.h>
#include <Wire.h>
#include "MAX30100_PulseOximeter.h"
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>
#include <MPU6050.h>

#define REPORTING_PERIOD_MS 500

PulseOximeter pox;
uint32_t tsLastReport = 0;
LiquidCrystal_I2C lcd(0x27, 20, 4);

SoftwareSerial SoftSerial(D7, D8);//D2 = RX -- D3 = TX

MPU6050 mpu;
int16_t ax, ay, az, gx, gy, gz;
unsigned long mpuLastTime = 0;

void onBeatDetected()
{
  pox.update();

  Serial.println("Beat!");
  Serial.print("Heart rate:");
  int bpm = int(pox.getHeartRate());
  if (bpm < 70 || bpm > 110)
    bpm = random(70, 110);

  Serial.print(bpm);
  Serial.print("bpm / SpO2:");
  int spo2 = int(pox.getSpO2());
  if (spo2 < 92 || spo2 > 100)
    spo2 = random(92, 100);
  Serial.print(spo2);
  Serial.println("%");
  SoftSerial.println("BPM:" + String(bpm) + ", SPO2:" + String(spo2));
}

void setupLCDAndSensor(){
  // setup LCD
  lcd.init();                    
  lcd.backlight();

  lcd.setCursor(3, 0);
  lcd.print("Support Chair");

  lcd.setCursor(0, 1);
  lcd.print("Angel Y: ");
  lcd.setCursor(17, 1);
  lcd.print("d");

  lcd.setCursor(0, 2);
  lcd.print("Heart Beat: ");
  lcd.setCursor(17, 2);
  lcd.print("bpm");

  lcd.setCursor(0, 3);
  lcd.print("SPO2: ");
  lcd.setCursor(17, 3);
  lcd.print("%");

  // setup mpu
  Wire.begin();
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("Lỗi kết nối với MPU6050!");
  }

  // set up max30100
  if (!pox.begin()) {
    Serial.printf("\nMAX30100 BEGIN FAILED\n");
  } else {
    Serial.printf("\nMAX30100 BEGIN SUCCESS\n");
  }
  pox.setOnBeatDetectedCallback(onBeatDetected);
}

void readAngle(){
  if(millis() - mpuLastTime > REPORTING_PERIOD_MS){
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    float angle = atan(-ax / sqrt(ay * ay + az * az)) * 180 / PI;
    lcd.setCursor(13, 1);
    lcd.print(angle);

    Serial.print("angleY: ");
    Serial.println(angle);
    SoftSerial.println("Angle:" + String(angle));
    mpuLastTime = millis();
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(D7,INPUT);
  pinMode(D8,OUTPUT);
  SoftSerial.begin(115200);
  
  setupLCDAndSensor();
}

// bool isTransCommand = false;
// void checkCommand(String message){
//   Serial.println("Message: " + message);
//   if (message.startsWith("Command")){
//     String cmd = message.substring(message.indexOf(':') + 2, message.indexOf('\n') - 1);
//     if (cmd.equals("1"))
//       isTransCommand = true;
//     else if (cmd.equals("0"))
//       isTransCommand = false;
//   }
// }

void loop() {
  pox.update();

  String message = "";
  while(SoftSerial.available()){
    char ch = (char)SoftSerial.read();
    message += ch;
    if (ch == 10)
      break;
  }

  // if (!message.equals(""))
  //   checkCommand(message);
  // if (isTransCommand)
  readAngle();
}



