// Potentiometer is connected to GPIO 34 (Analog ADC1_CH6) 
const int throttlePin = 32;
#define THROTTLE_PIN 32
#define WINDOW_SIZE 5
int throttleMin = 900;
int throttleMax = 3100;
int throttleRange = (throttleMax - throttleMin);
int adcRaw = 0;
int throttleValue = 0;
int INDEX = 0;
int VALUE = 0;
int SUM = 0;
int READINGS[WINDOW_SIZE];
int AVERAGED = 0;

// the number of the LED pin
const int ledPin = 33;  // 16 corresponds to GPIO16
// setting PWM properties
const int freq = 20000;
const int ledChannel = 0;
const int resolution = 8;
 

void setup() {
  Serial.begin(115200);

  // configure LED PWM functionalitites
  ledcSetup(ledChannel, freq, resolution);
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(ledPin, ledChannel);

  delay(1000);
}

void loop() {

  SUM = SUM - READINGS[INDEX];       // Remove the oldest entry from the sum
  VALUE = analogRead(THROTTLE_PIN);        // Read the next sensor value
  READINGS[INDEX] = VALUE;           // Add the newest reading to the window
  SUM = SUM + VALUE;                 // Add the newest reading to the sum
  INDEX = (INDEX+1) % WINDOW_SIZE;   // Increment the index, and wrap to 0 if it exceeds the window size
  AVERAGED = SUM / WINDOW_SIZE;      // Divide the sum of the window by the window size for the result
  throttleValue = ((AVERAGED - throttleMin) * 1000) / throttleRange;
  if (throttleValue < 0) { throttleValue = 0; }
  Serial.println(throttleValue);

  ledcWrite(ledChannel, throttleValue / 4);

  delay(100);
}