// the number of the LED pin
const int ledPin = 13;  // 16 corresponds to GPIO16
const int ledDir = 12;
//const int ledEn = 14;
const int switchPin = 14;

// setting PWM properties
const int freq = 4000; // foot 200, thigh 4000
const int ledChannel = 0;
const int resolution = 8;

long delay_time = 15000;

void setup(){
  Serial.begin(115200);
  // configure LED PWM functionalitites
  //pinMode(ledEn, OUTPUT);
  pinMode(ledDir, OUTPUT);
  pinMode(switchPin, INPUT_PULLUP);
  //digitalWrite(ledEn, LOW);

  ledcSetup(ledChannel, freq, resolution);
  
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(ledPin, ledChannel);
  
  digitalWrite(ledDir, LOW);
  ledcWrite(ledChannel, 20);
  delay(2000);

  while (digitalRead(switchPin) == HIGH)
  {
    Serial.printf("In while!\n");
    digitalWrite(ledDir, HIGH);
    ledcWrite(ledChannel, 20);
    delay(5);
  }
  Serial.printf("Out while!\n");
  ledcWrite(ledChannel, 0);
  delay(1000);
}
 
void loop(){

  digitalWrite(ledDir, LOW);
  int dutyCycle = 20;
  ledcWrite(ledChannel, dutyCycle);
  
  delay(delay_time);
  digitalWrite(ledDir, HIGH);
  ledcWrite(ledChannel, dutyCycle);

  delay(delay_time);
}