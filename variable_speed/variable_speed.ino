#include "PINDEF.h"
#include "data_maps.h"
#include <EEPROM.h>

// Global BLE APP variables
int batteryLevel = 12;
bool reverseDirection = false;
bool reverseSteering = false;
int maxPower = 100;
int reversePowerMax = 20;
int throttleMaxPowerPos = 100;
int throttleVsPowerMap = 50;
bool updatedSettings = false;

int batteryCellCountHW = 3;
bool reverseDirectionHW = false;
bool reverseSteeringHW = false;
int maxPowerHW = 100;
int reversePowerMaxHW = 20;
int throttleMaxPowerPosHW = 100;
int throttleVsPowerMapHW = 50;
// END

int V_BAT   = 0;
int V_M     = 0;
int I_BAT   = 0;
int I_M     = 0;
int Temp_1 = 0;
int Temp_2 = 0;
int Temp_3 = 0;
int Temp_4 = 0;
int Temp_5 = 0;
int SOC = 100;

#define EEPROM_SIZE 7

#include "ble_app.h"

// Potentiometer is connected to GPIO 34 (Analog ADC1_CH6) 
// const int throttlePin = 32;
// #define THROTTLE_PIN 36
// #define WINDOW_SIZE 5
int throttleMin = 950;
int throttleMax = 3100;
int throttleRange = (throttleMax - throttleMin);
int adcRaw = 0;
int throttleValue = 0;
int INDEX = 0;
int VALUE = 0;
int SUM = 0;
int READINGS[WINDOW_SIZE];
int AVERAGED = 0;

bool BUS_ON = false;

// Voltage dividers
// VM = Vadc x xVMADC
int R1 = 100000;
int R2 = 10000;
// xVMADC = (3300 / 4095) * (R1+R2) / R2; 0.8059 * 5.7
int xVMADC = 10; // 8865;

// the number of the LED pin
// const int STATUS = 33;  // 16 corresponds to GPIO16
// setting PWM properties
const int freq = 20000;
const int resolution = 8;
const int throttle_channel = 0;
int throttle_out = 0;

const int s_freq = 1000;
const int s_resolution = 8;
const int r_channel = 2;
const int l_channel = 4;
int s_time = millis();
int s_on_time = 1000;

int send_app_timer = millis();
int send_app_interval = 1000;

// const int DIR_PIN = 34; // DO
// const int VREF_PIN = 32; // DO
// const int SLEEP_PIN = 25; // DO
// const int STATUS_PIN = 19; // DO
// const int SNSOUT_PIN = 26; // INPUT
// const int nFAULT_PIN = 27; // INPUT
// const int VM_ADC_PIN = 37; // INPUT
// const int iM_ADC_PIN = 39; // INPUT

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

  EEPROM.begin(EEPROM_SIZE);

  batteryCellCountHW = EEPROM.read(BATTERY_VOLTAGE_INDEX);
  reverseDirectionHW = EEPROM.read(DIRECTION_SETTING_INDEX);
  reverseSteeringHW = EEPROM.read(STEERING_SETTING_INDEX);
  maxPowerHW = EEPROM.read(MAX_POWER_INDEX);
  reversePowerMaxHW = EEPROM.read(REVERSE_POWER_INDEX);
  throttleMaxPowerPosHW = EEPROM.read(THROTTLE_MAX_POWER_POS_INDEX);
  throttleVsPowerMapHW = EEPROM.read(THROTTLE_VS_POWER_MAP_INDEX);
  
  ble_setup();
  Serial.println("Setup complete. Listening for BLE updates...");

  // configure LED PWM functionalitites
  ledcSetup(throttle_channel, freq, resolution);
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(PWM_MTR_PIN, throttle_channel);
  // initialize to zero
  ledcWrite(throttle_channel, 0 );

  // configure LED PWM functionalitites
  ledcSetup(r_channel, s_freq, s_resolution);
  ledcSetup(l_channel, s_freq, s_resolution);
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(R_DRIVE_PIN, r_channel);
  ledcAttachPin(L_DRIVE_PIN, l_channel);
  // initialize to zero
  ledcWrite(r_channel, 0 );
  ledcWrite(l_channel, 0 );
  
  // pinMode(DIR_PIN,OUTPUT);
  pinMode(VREF_PIN,OUTPUT);  // VREF
  pinMode(SLEEP_PIN,OUTPUT);
  pinMode(STATUS_PIN,OUTPUT);  // SNSOUT
  pinMode(DIR_MTR_PIN,OUTPUT);
  digitalWrite(DIR_MTR_PIN, LOW);  

  pinMode(CONN_PIN,OUTPUT);  // SNSOUT
  digitalWrite(CONN_PIN, LOW);

  // pinMode(R_DRIVE_PIN,OUTPUT);  // SNSOUT
  // pinMode(L_DRIVE_PIN,OUTPUT);  // SNSOUT
  // digitalWrite(R_DRIVE_PIN, LOW);
  // digitalWrite(L_DRIVE_PIN, LOW);

  pinMode(ON_PIN,INPUT);  // SNSOUT
  pinMode(SNSOUT_PIN,INPUT);  // SNSOUT
  pinMode(nFAULT_PIN,INPUT);  // nFAULT
  pinMode(REV_PIN, INPUT);  // Reverse input

  pinMode(VB_ADC_PIN,INPUT);  // Battery (system) Input voltage
  pinMode(IB_ADC_PIN,INPUT);  // Battery (system) Input current
  pinMode(VM_ADC_PIN,INPUT);  // Motor voltage ie post contactor
  pinMode(iM_ADC_PIN,INPUT);  // MCU current

  delay(1000);
}

void loop() {

  if ( (updatedSettings == true) && (throttle_out < 100) ) {
    Serial.printf("New Battery Voltage: %d\r\n", batteryLevel); // example of referencing the updated

    if (batteryLevel < 13) {
      batteryCellCountHW = 3;
    } else if (batteryLevel < 21) {
      batteryCellCountHW = 5;
    } else {
      batteryCellCountHW = 6;
    }    
    reverseDirectionHW = reverseDirection;
    reverseSteeringHW = reverseSteering;
    maxPowerHW = maxPower;
    reversePowerMaxHW = reversePowerMax;
    throttleMaxPowerPosHW = throttleMaxPowerPos;
    throttleVsPowerMapHW = throttleVsPowerMap;

    EEPROM.write(BATTERY_VOLTAGE_INDEX, batteryCellCountHW);
    EEPROM.write(DIRECTION_SETTING_INDEX, reverseDirectionHW);
    EEPROM.write(STEERING_SETTING_INDEX, reverseSteeringHW);
    EEPROM.write(MAX_POWER_INDEX, maxPowerHW);
    EEPROM.write(REVERSE_POWER_INDEX, reversePowerMaxHW);
    EEPROM.write(THROTTLE_MAX_POWER_POS_INDEX, throttleMaxPowerPosHW);
    EEPROM.write(THROTTLE_VS_POWER_MAP_INDEX, throttleVsPowerMapHW);

    EEPROM.commit();
    
    updatedSettings = false;
  }

  // Serial.println(maxPowerHW);
  
  // Get throttle input
  throttleValue = read_throttle();

  // Get ADCs ie voltage, currents, and temps
  V_BAT = read_voltage(VB_ADC_PIN);
  V_M = read_voltage(VM_ADC_PIN);
  I_BAT = read_current_bat(IB_ADC_PIN);
  I_M = read_current_mtr(iM_ADC_PIN);
  Temp_1 = read_temp(T1_PIN);
  Temp_2 = read_temp(T2_PIN);
  Temp_3 = read_temp(T3_PIN);
  Temp_4 = read_temp(T4_PIN);
  Temp_5 = read_temp(T5_PIN);

  // Soft start / pre-charge
  bool on_switch = digitalRead(ON_PIN);
  if ( V_BAT < 6000 ) {
    digitalWrite(CONN_PIN, LOW);
    BUS_ON = false;
  } else if ( on_switch && (BUS_ON == false) && (V_M > V_BAT - 2000) ) {
    digitalWrite(CONN_PIN, HIGH);
    BUS_ON = true;
    delay(100);
    s_time = millis();
  } else {
    digitalWrite(CONN_PIN, LOW);
    BUS_ON = false;
  }

  // SOC
  int cellVoltage = V_BAT / batteryCellCountHW;
  for (int x = 0; x < 101; x++) {
    if( cellVoltage > mapSOC[x] ) {
      SOC = x;
    }
  }
  
  int s_current_time = millis() - s_time;
  
  if (BUS_ON) {
    // Send throttle to MCU
    throttle_out = set_throttle(throttleValue);
    // Write to PWM_MTR_PIN

    digitalWrite(SLEEP_PIN, HIGH);
    digitalWrite(STATUS_PIN, HIGH);

    bool reverse_in = digitalRead(REV_PIN);
    if( reverse_in ) {
      digitalWrite(DIR_MTR_PIN, HIGH);
      ledcWrite(throttle_channel, throttle_out / 4);
    } else {
      digitalWrite(DIR_MTR_PIN, LOW);
      if( throttle_out > (reversePowerMaxHW*10) ) {
        throttle_out = reversePowerMaxHW*10;
      }   
      // throttle_out = (throttle_out * reversePowerMaxHW) / 100;
      ledcWrite(throttle_channel, throttle_out / 4);
    }

    // delay(100);
    // digitalWrite(DIR_PIN, HIGH);
    digitalWrite(VREF_PIN, HIGH);

    if ( BUS_ON ) { 
      if ( s_current_time > 8000 ) {
        ledcWrite(r_channel, 00);
        ledcWrite(l_channel, 00);      
      } else if ( s_current_time > 6000 ){  // Left
        ledcWrite(r_channel, 00);
        ledcWrite(l_channel, 50);
      } else if ( s_current_time > 4000 ){
        ledcWrite(r_channel, 00);
        ledcWrite(l_channel, 00);
      } else if ( s_current_time > 2000 ){  // Right
        ledcWrite(r_channel, 50);
        ledcWrite(l_channel, 00);
      }
    }
  }

  if ( (millis() - send_app_timer) > send_app_interval ) {
    Serial.println("Data sent");
    send_app_timer = millis();
  }

  // Serial.print(throttleValue);
  // Serial.print(",");
  // Serial.println(throttle_out);
  // Serial.print(",");
  // Serial.print(V_BAT);
  // Serial.print(",");
  // Serial.print(V_M);
  // Serial.print(",");
  // Serial.print(I_BAT);
  // Serial.print(",");
  // Serial.print(s_current_time);
  // Serial.println();
  
  // Serial.print(Temp_1);
  // Serial.print(" , ");
  // Serial.print(Temp_2);
  // Serial.print(" , ");
  // Serial.print(Temp_3);
  // Serial.print(" , ");
  // Serial.print(Temp_4);
  // Serial.print(" , ");
  // Serial.print(Temp_5);
  // Serial.print(" , ");
  // Serial.println(throttle_out);

  // VMmv = analogRead(VM_ADC_PIN) * xVMADC / 1000;
  // Serial.print(" - Vmotor: ");
  // Serial.println(VMmv);
  // IM = analogRead(iM_ADC_PIN);

  // delay(50);
}

int read_throttle(){
  int result;

  SUM = SUM - READINGS[INDEX];       // Remove the oldest entry from the sum
  VALUE = analogRead(THROTTLE_PIN);        // Read the next sensor value
  READINGS[INDEX] = VALUE;           // Add the newest reading to the window
  SUM = SUM + VALUE;                 // Add the newest reading to the sum
  INDEX = (INDEX+1) % WINDOW_SIZE;   // Increment the index, and wrap to 0 if it exceeds the window size
  AVERAGED = SUM / WINDOW_SIZE;      // Divide the sum of the window by the window size for the result
  result = ((AVERAGED - throttleMin) * 1000) / throttleRange;
  if (result < 0) { result = 0; }
  // Serial.print("Thrt: ");
  // Serial.print(throttleValue);

  return result;
}

int set_throttle(int x){

  // Calculate the error (difference between setpoint and actual value)
  // Limit the change in PWM value to maxChange
  int deltaPWMValue = constrain(x - previousPWMValue, -maxChange, maxChange);
  int newPWMValue = previousPWMValue + deltaPWMValue;
  // Update the previous PWM value for the next iteration
  previousPWMValue = newPWMValue;

  return newPWMValue;
}

int read_temp(int x_pin){
  int raw_read = 0;
  int calculated = 0;
  int out_temp = 0;
  raw_read = analogRead(x_pin);
  calculated = raw_read * 0.806;
  for (int x = 0; x < 126; x++) {
    if( calculated < mapTemp[x] ) {
      out_temp = x;  
    }
  }
  return out_temp;
}

int read_voltage(int x_pin){
  int raw_read = 0;
  int calculated = 0;
  raw_read = analogRead(x_pin);
  calculated = raw_read * xVMADC;
  return calculated;
}

int read_current_bat(int x_pin){
  int raw_read = 0;
  int calculated = 0;
  raw_read = analogRead(x_pin);
  calculated = raw_read;
  return raw_read;
}

int read_current_mtr(int x_pin){
  int raw_read = 0;
  int calculated = 0;
  raw_read = analogRead(x_pin);
  calculated = raw_read * xVMADC / 100;
  return raw_read;
}


