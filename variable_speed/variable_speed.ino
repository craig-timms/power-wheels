// Potentiometer is connected to GPIO 34 (Analog ADC1_CH6) 
const int throttlePin = 32;
#define THROTTLE_PIN 36
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

// Voltage dividers
// VM = Vadc x xVMADC
int R1 = 100000;
int R2 = 10000;
// xVMADC = (3300 / 4095) * (R1+R2) / R2; 0.8059 * 5.7
int xVMADC = 10000; // 8865;

// the number of the LED pin
const int ledPin = 33;  // 16 corresponds to GPIO16
// setting PWM properties
const int freq = 20000;
const int ledChannel = 0;
const int resolution = 8;

// const int DIR_PIN = 34; // DO
const int VREF_PIN = 32; // DO

const int SLEEP_PIN = 25; // DO
const int STATUS_PIN = 19; // DO

const int SNSOUT_PIN = 26; // INPUT
const int nFAULT_PIN = 27; // INPUT
const int VM_ADC_PIN = 37; // INPUT
const int iM_ADC_PIN = 39; // INPUT

int VMmv = 0;
int IM = 0;

bool dirMotor = false;
bool nSLEEP = false;

// PI controller
// Define the maximum change in PWM value per iteration
const int tDelay = 50;
const int periods = 1000 / tDelay;
const int slew = 1000; // full range time
const int maxChange = ( 1000000 / slew ) / periods;
// Variables to store the previous PWM value
int previousPWMValue = 0;

void setup() {
  Serial.begin(115200);

  // configure LED PWM functionalitites
  ledcSetup(ledChannel, freq, resolution);
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(ledPin, ledChannel);
  
  // pinMode(DIR_PIN,OUTPUT);
  pinMode(VREF_PIN,OUTPUT);  // VREF
  pinMode(SLEEP_PIN,OUTPUT);
  pinMode(STATUS_PIN,OUTPUT);  // SNSOUT
  
  pinMode(SNSOUT_PIN,INPUT);  // SNSOUT
  pinMode(nFAULT_PIN,INPUT);  // nFAULT
  // pinMode(VM_ADC_PIN,INPUT);  // VMadc
  pinMode(iM_ADC_PIN,INPUT);  // iMadc

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
  // Serial.print("Thrt: ");
  // Serial.print(throttleValue);

  // Calculate the error (difference between setpoint and actual value)
  // Limit the change in PWM value to maxChange
  int deltaPWMValue = constrain(throttleValue - previousPWMValue, -maxChange, maxChange);
  int newPWMValue = previousPWMValue + deltaPWMValue;
  // Update the previous PWM value for the next iteration
  previousPWMValue = newPWMValue;

  ledcWrite(ledChannel, newPWMValue / 4);

  digitalWrite(SLEEP_PIN, HIGH);
  digitalWrite(STATUS_PIN, HIGH);

  // delay(100);
  // digitalWrite(DIR_PIN, HIGH);
  digitalWrite(VREF_PIN, HIGH);

  VMmv = analogRead(VM_ADC_PIN) * xVMADC / 1000;
  // Serial.print(" - Vmotor: ");
  // Serial.println(VMmv);
  IM = analogRead(iM_ADC_PIN);

  Serial.print(throttleValue);
  Serial.print(",");
  Serial.print(newPWMValue);
  Serial.print(",");
  Serial.print(VMmv);
  Serial.print(",");
  Serial.print(IM);
  Serial.println();

  delay(50);
}