// the number of the LED pin
const int ledPin = 14;  // 16 corresponds to GPIO16
const int ledDir = 12;
const int ledEn = 13;
const int switchPin = 15;

// setting PWM properties
const int freq = 4000; // foot 200, thigh 2000
const int ledChannel = 0;
const int resolution = 8;

long delay_time = 15000;

void setup(){
  Serial.begin(115200);
  // configure LED PWM functionalitites
  pinMode(ledEn, OUTPUT);
  pinMode(ledDir, OUTPUT);
  pinMode(switchPin, INPUT_PULLUP);
  digitalWrite(ledEn, LOW);

  ledcSetup(ledChannel, freq, resolution);
  
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(ledPin, ledChannel);

  while (digitalRead(switchPin) == HIGH)
  {
    digitalWrite(ledDir, LOW);
    ledcWrite(ledChannel, 200);
    delay(5);
  }
  ledcWrite(ledChannel, 0);
}
 
void loop(){

  digitalWrite(ledDir, HIGH);
  int dutyCycle = 25;
  ledcWrite(ledChannel, dutyCycle);
  
  delay(delay_time);
  digitalWrite(ledDir, LOW);
  ledcWrite(ledChannel, dutyCycle);

  delay(delay_time);
}