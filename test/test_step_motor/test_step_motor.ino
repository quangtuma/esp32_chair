// the number of the LED pin
const int ledPin = 13;  // 16 corresponds to GPIO16
const int ledDir = 12;
//const int ledEn = 14;
const int switchPin = 14;

// setting PWM properties
const int freq = 3000; // foot 200, thigh 4000
const int ledChannel = 0;
const int resolution = 8;

long delay_time = 5000;

void setup(){
  Serial.begin(115200);
  // configure LED PWM functionalitites
  pinMode(ledPin, OUTPUT);
  pinMode(ledDir, OUTPUT);
  //pinMode(switchPin, INPUT_PULLUP);
  //digitalWrite(ledEn, LOW);

  ledcSetup(ledChannel, freq, resolution);
  
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(ledPin, ledChannel);
}
 
void loop(){
  
  int dutyCycle = 200;
  digitalWrite(ledDir, LOW);
  ledcWrite(ledChannel, dutyCycle);
  
  // delay(delay_time);
  // digitalWrite(ledDir, LOW);
  // ledcWrite(ledChannel, dutyCycle);

  // delay(delay_time);
}