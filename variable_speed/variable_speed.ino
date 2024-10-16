#include "PINDEF.h"
#include "data_maps.h"
// #include <EEPROM.h>
#include <Preferences.h>
Preferences preferences;
#define DEBUG_PLOTTER 0
#define DEBUG_STATES 0

#define MIN_NAME_LEN 4
#define MAX_NAME_LEN 32
#define MAX_NAME_INTERNAL_LEN MAX_NAME_LEN+1

// Global BLE APP variables
// udatedSettings flag variables
bool updatedSettings = false;
int batteryLevel = 12;
bool reverseDirection = false;
bool reverseSteering = false;
int maxPower = 100;
int reversePowerMax = 20;
int throttleMaxPowerPos = 100;
int throttleVsPowerMap = 50;

int batteryCellCountHW = 3;
bool reverseDirectionHW = false;
bool reverseSteeringHW = false;
int maxPowerHW = 100;
int reversePowerMaxHW = 20;
int throttleMaxPowerPosHW = 100;
int throttleVsPowerMapHW = 50;
char deviceNameHW[MAX_NAME_INTERNAL_LEN] = {0};

// updatedControls flag variables
bool updatedControls = false;
bool outOfRangeEnable = true;
bool eStop = false;
bool remoteEnable = false;

bool updatedControlsHW = false;
bool outOfRangeEnableHW = true;
bool eStopHW = false;
bool remoteEnableHW = false;
// END



// BATTERY_VOLTAGE_INDEX);
// DIRECTION_SETTING_INDEX);
// STEERING_SETTING_INDEX);
// MAX_POWER_INDEX);
// REVERSE_POWER_INDEX);
// THROTTLE_MAX_POWER_POS_INDEX);
// THROTTLE_VS_POWER_MAP_INDEX);

// updatedThrottle flags
bool updatedThrottle = false; // This tells you there is a new command from the app
int throttleCommand = 0;

int throttleCommandHw = 0;// IDK if you need this or not
// END

// Send HW data to BT app timer
int timerApp = millis();
int timeApp = 1000;

int vehicleState = 0;
int vehicleStatePrevious = -1;
int vehicleStateNext = 2;
// 0 - startup
// 1 - reset
// 2 - normal
// 3 - remote
// 4 - fault

int hardFault = 0;
// 0 - no fault
// 1 - temp1
// 2 - temp2
// 3 - temp3
// 4 - temp4
// 5 - temp5
// 6 - OC
// 7 - OV
// 8 - MCU
// 9 - range
// 10 - eStop
// 11 - lowBatt

bool reverse = false;
bool reverseGPIOold = false;

int timerStartup = millis();
int timeStartup = 4000;
int timerContactor = millis();
int timeContactor = 1000;
int timerReset = millis();
int timeReset = 100;
int timerNormal = millis();
int timeNormal = 1000;
int timerFault = millis();
int timeFault = 1000;
int timerPlotter = millis();
int timePlotter = 50;
int timerThrottle = millis();
int timeThrottle = 100;

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

// Throttle adc input stuff
int throttleMin = 950;
int throttleMax = 3000;
int throttleRange = (throttleMax - throttleMin);
int adcRaw = 0;
int throttleValue = 0;

int INDEX = 0;
int VALUE = 0;
int SUM = 0;
int READINGS[WINDOW_SIZE];
int AVERAGED = 0;

int INDEXthrottle = 0;
int VALUEthrottle = 0;
int SUMthrottle = 0;
int READINGSthrottle[WINDOW_SIZE];
int AVERAGEDthrottle = 0;

#define WINDOW_SIZE_vbat 20
int INDEXvbat = 0;
int VALUEvbat = 0;
int SUMvbat = 0;
int READINGSvbat[WINDOW_SIZE_vbat];
int AVERAGEDvbat = 0;

#define WINDOW_SIZE_ibat 10
unsigned long INDEXibat = 0;
unsigned long VALUEibat = 0;
unsigned long SUMibat = 0;
unsigned long READINGSibat[WINDOW_SIZE_ibat];
unsigned long AVERAGEDibat = 0;

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

// Steering variables
const int s_freq = 1000;
const int s_resolution = 8;
const int r_channel = 2;
const int l_channel = 4;
int s_time = millis();
int s_on_time = 1000;

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
const int slewN = 20000; // full range time
const int slewP = 10000; // full range time
const int maxChangeN = ( 1000000 / slewN ) / periods;
const int maxChangeP = ( 1000000 / slewP ) / periods;
// Variables to store the previous PWM value
int previousPWMValue = 0;

void setup() {
  Serial.begin(115200);

  // EEPROM.begin(EEPROM_SIZE);

  // batteryCellCountHW = EEPROM.read(BATTERY_VOLTAGE_INDEX);
  // reverseDirectionHW = EEPROM.read(DIRECTION_SETTING_INDEX);
  // reverseSteeringHW = EEPROM.read(STEERING_SETTING_INDEX);
  // maxPowerHW = EEPROM.read(MAX_POWER_INDEX);
  // reversePowerMaxHW = EEPROM.read(REVERSE_POWER_INDEX);
  // throttleMaxPowerPosHW = EEPROM.read(THROTTLE_MAX_POWER_POS_INDEX);
  // throttleVsPowerMapHW = EEPROM.read(THROTTLE_VS_POWER_MAP_INDEX);

  preferences.begin("my-app", false);

  batteryCellCountHW = preferences.getInt("vbat",3);
  reverseDirectionHW = preferences.getBool("rev",false);
  reverseSteeringHW = preferences.getBool("steer",false);
  maxPowerHW = preferences.getInt("maxP",100);
  reversePowerMaxHW = preferences.getInt("maxR",20);
  throttleMaxPowerPosHW = preferences.getInt("maxP",100);
  throttleVsPowerMapHW = preferences.getInt("map",50);

  size_t nameLen = preferences.getBytesLength("name");
  Serial.printf("Name length: %d\r\n", nameLen);
  if (nameLen < MIN_NAME_LEN) {
    const char * defaultName = "My Smooth Scoot";
    strcpy(deviceNameHW, defaultName);
  } else {
    preferences.getBytes("name", deviceNameHW, MAX_NAME_LEN);
  }
  
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

  ble_checkToReconnect();

  // DEBUG what is in EEPROM
  if ( vehicleStatePrevious == -1 ) {
    Serial.println();
    Serial.printf("Cell count: %d\r\n", batteryCellCountHW);
    Serial.printf("Reverse change: %d\r\n", reverseDirectionHW);
    Serial.printf("Steering change: %d\r\n", reverseSteeringHW);
    Serial.printf("Max power: %d\r\n", maxPowerHW);
    Serial.printf("Reverse power: %d\r\n", reversePowerMaxHW);
    Serial.printf("Throttle position max: %d\r\n", throttleMaxPowerPosHW);
    Serial.printf("Throttle map: %d\r\n", throttleVsPowerMapHW);
    Serial.println();
    vehicleState = 0;
    vehicleStatePrevious = 0;
  }

  if ( vehicleState == 4 ) {
    vehicleState == 4; // fault
  } else if ( updatedControls ) {
    if ( eStop ) {
      vehicleState = 4; // fault
      timerFault = millis();
      hardFault = 10;
      eStopHW = eStop;
    } else if ( (outOfRangeEnable != outOfRangeEnableHW) || (remoteEnable != remoteEnableHW) ) {
      vehicleState = 1; // reset
      timerReset = millis();
      outOfRangeEnableHW = outOfRangeEnable;
      remoteEnableHW = remoteEnable;
    }
    updatedControls = false;
  } else if ( updatedSettings ) {
    vehicleState = 1; // reset
    timerReset = millis();
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

    // EEPROM.write(BATTERY_VOLTAGE_INDEX, batteryCellCountHW);
    // EEPROM.write(DIRECTION_SETTING_INDEX, reverseDirectionHW);
    // EEPROM.write(STEERING_SETTING_INDEX, reverseSteeringHW);
    // EEPROM.write(MAX_POWER_INDEX, maxPowerHW);
    // EEPROM.write(REVERSE_POWER_INDEX, reversePowerMaxHW);
    // EEPROM.write(THROTTLE_MAX_POWER_POS_INDEX, throttleMaxPowerPosHW);
    // EEPROM.write(THROTTLE_VS_POWER_MAP_INDEX, throttleVsPowerMapHW);
    // EEPROM.commit();
     
    preferences.putInt("vbat",batteryCellCountHW);
    preferences.putBool("rev",reverseDirectionHW);
    preferences.putBool("steer",reverseSteeringHW);
    preferences.putInt("maxP",maxPowerHW);
    preferences.putInt("maxR",reversePowerMaxHW);
    preferences.putInt("maxP",throttleMaxPowerPosHW);
    preferences.putInt("map",throttleVsPowerMapHW);
    // name is handled directly in the BLE function so we dont pass strings around

    Serial.println();
    Serial.println("Settings changed by app:");
    Serial.printf("Cell count: %d\r\n", batteryCellCountHW);
    Serial.printf("Reverse change: %d\r\n", reverseDirectionHW);
    Serial.printf("Steering change: %d\r\n", reverseSteeringHW);
    Serial.printf("Max power: %d\r\n", maxPowerHW);
    Serial.printf("Reverse power: %d\r\n", reversePowerMaxHW);
    Serial.printf("Throttle position max: %d\r\n", throttleMaxPowerPosHW);
    Serial.printf("Throttle map: %d\r\n", throttleVsPowerMapHW);
    Serial.println();    
    
    updatedSettings = false;
    delay(100);
  }

  // DEBUG states
  if ( DEBUG_STATES && ( vehicleState != vehicleStatePrevious ) ) {
    Serial.printf("State change (1): %d\r\n", vehicleState);
    vehicleStatePrevious = vehicleState;
  }
  
  // Get ADCs ie voltage, currents, and temps
  V_BAT = read_vbat();
  V_M = read_voltage(VM_ADC_PIN);
  I_BAT = read_current_bat(IB_ADC_PIN);
  I_M = read_current_mtr(iM_ADC_PIN);
  Temp_1 = read_temp(T1_PIN);
  Temp_2 = read_temp(T2_PIN);
  Temp_3 = read_temp(T3_PIN);
  Temp_4 = read_temp(T4_PIN);
  Temp_5 = read_temp(T5_PIN);

  // SOC
  // First power up might fault because of battery
  // set correctly in app and power cycle vehicle
  int cellVoltage = V_BAT / batteryCellCountHW;
  for (int x = 0; x < 101; x++) {
    if( cellVoltage > mapSOC[x] ) {
      SOC = x;
    }
  }
  if ( (SOC < 20) && (vehicleState != 0) ) {
    vehicleState = 4;
    hardFault = 11;
  }

  // DEBUG states
  if ( DEBUG_STATES && ( vehicleState != vehicleStatePrevious ) ) {
    Serial.printf("State change (2): %d\r\n", vehicleState);
    vehicleStatePrevious = vehicleState;
  }

  int steer = 1; // left nothing right
  bool on_switch = digitalRead(ON_PIN);

  if ( vehicleState == 4 ) { // fault
    throttleValue = 0;
    if ( throttle_out < 10 ) {
      digitalWrite(SLEEP_PIN, LOW);
      digitalWrite(STATUS_PIN, LOW);
      // TODO
      // delay 100
      // turn off battery
      // maybe EEPROM fault
    }
    if ( throttle_out == 0 ) {
      Serial.println("Fault Shutdown");
      digitalWrite(CONN_PIN, LOW);
    }
    // No steering
    ledcWrite(r_channel, 00);
    ledcWrite(l_channel, 00);
  } if ( vehicleState == 0 ) { // startup
    throttleValue = 0;
    digitalWrite(SLEEP_PIN, HIGH);
    digitalWrite(STATUS_PIN, HIGH);

    // Soft start / pre-charge
    if ( V_BAT < 4000 ) { // Programming connected
      digitalWrite(CONN_PIN, LOW);
      BUS_ON = false;
      timerContactor = millis();
      timerStartup = millis();      
    } else if ( !on_switch ) {
      // shut down
      // Serial.println("Should be shutting down right now... ");
      // if ( throttle_out == 0 ) {
      //   digitalWrite(CONN_PIN, LOW);          
      // }
      // TODO
    } else if ( ((millis()-timerStartup) > timeStartup) && BUS_ON ) {
      Serial.printf("Time up startup: %d\r\n", millis()-timerStartup);
      if ( remoteEnableHW ) {
        vehicleState = 3;
      } else {
        vehicleState = 2;
      }
      Serial.println("Out of startup");
    } else if ( ((millis()-timerContactor) > timeContactor) && (!BUS_ON) && (V_M > V_BAT - 6000) ) {
      digitalWrite(CONN_PIN, HIGH);
      BUS_ON = true;
      delay(100);
      s_time = millis();
      Serial.println("Contactor closed");
    }
    // No steering
    ledcWrite(r_channel, 00);
    ledcWrite(l_channel, 00);
  } else if ( vehicleState == 1 ) { // reset
    throttleValue = 0;
    digitalWrite(SLEEP_PIN, HIGH);
    digitalWrite(STATUS_PIN, HIGH);
    // if ( (millis()-timerReset) > timeReset ) {
    if ( ( (millis()-timerReset) > timeReset ) && ( throttle_out == 0 ) ) {
      if ( ( throttle_out == 0 ) && ( !on_switch ) ) {
        Serial.println("Switch Shutdown");
        digitalWrite(CONN_PIN, LOW);
      } else if ( remoteEnableHW ) {
        vehicleState = 3;
      } else {
        vehicleState = 2;
      }
    }
    // No steering
    ledcWrite(r_channel, 00);
    ledcWrite(l_channel, 00);
  //
  //
  // NORMAL STATE
  //
  //
  } else if ( vehicleState == 2 ) {
    if ( !on_switch ) {
      vehicleState = 1;
    }
    // Get throttle input
    throttleValue = read_throttle();
    digitalWrite(SLEEP_PIN, HIGH);
    digitalWrite(STATUS_PIN, HIGH);
    digitalWrite(VREF_PIN, HIGH);
    // Get HW reverse input    
    bool reverseGPIO = digitalRead(REV_PIN);
    reverse = !reverseGPIO; // Pull up resistor so reverse switch is low
    if ( reverseGPIO != reverseGPIOold ) {
      vehicleState = 1; // reset
      timerReset = millis();
      throttleValue = 0;      
    }
    reverseGPIOold = reverseGPIO;
    
    // No steering in HW control
    ledcWrite(r_channel, 00);
    ledcWrite(l_channel, 00);
  //
  //
  // REMOTE STATE
  //
  //
  } else if ( vehicleState == 3 ) {
    if ( !on_switch ) {
      vehicleState = 1;
      timerReset = millis();
    }
    // TODO
    if ( ( (millis() - timerThrottle ) > timeThrottle ) ) { // updatedThrottle
      
      // Should not need this
      // Zero crossing issue on app side
      // Sometimes at zero crossing throttleCommand jumps to 100
      // This catches that as an outlier
      if ( ( throttleCommand * 10 > throttleValue + 600 ) || ( throttleCommand * 10 > throttleValue + 600 ) ) {
        // do not take command
      } else {
        throttleValue = throttleCommand * 10;
        updatedThrottle = false;
      // Serial.println(throttleCommand);
      }
      
      if ( (throttleValue < 0) && !reverse ) {
        throttleValue = 0;
        reverse = true;
        vehicleState = 1; // reset
        timerReset = millis();
      } else if ( throttleValue < 0 ) {
        throttleValue = throttleValue * -1;
      } else if ( (throttleValue >= 0) && reverse ) {
        throttleValue = 0;
        reverse = false;
        vehicleState = 1; // reset
        timerReset = millis();
      }
      timerThrottle = millis();
    }

    // TODO
    // steer = value from app // 0 - left, 2 - right
    if ( reverseSteeringHW ) {
      if ( steer == 0 ) {
        steer = 2;
      } else if ( steer == 2 ) {
        steer = 0;
      }
    }
    // Sending 50% duty, not sure if best thing to do
    if ( steer == 1 ) {
      ledcWrite(r_channel, 00);
      ledcWrite(l_channel, 00);
    } else if ( steer = 0 ) { 
      // Left
      ledcWrite(r_channel, 00);
      ledcWrite(l_channel, 50);
    } else if ( steer = 2 ) {
      // Right
      ledcWrite(r_channel, 50);
      ledcWrite(l_channel, 00);
    }
    digitalWrite(SLEEP_PIN, HIGH);
    digitalWrite(STATUS_PIN, HIGH);
    digitalWrite(VREF_PIN, HIGH);    
  }

  bool reverse_out = reverse;
  if ( reverseDirectionHW ) {
    reverse_out = !reverse;
  }

  if( !reverse_out ) {
    digitalWrite(DIR_MTR_PIN, HIGH);
    throttle_out = set_throttle(throttleValue,100);
    ledcWrite(throttle_channel, throttle_out / 4);
  } else {
    digitalWrite(DIR_MTR_PIN, LOW);
    throttle_out = set_throttle(throttleValue,reversePowerMaxHW);
    // TODO move max into function(throttleValue, max, map, powers)
    // if( throttle_out > (reversePowerMaxHW*10) ) {
    //   throttle_out = reversePowerMaxHW*10;
    // }   
    // throttle_out = (throttle_out * reversePowerMaxHW) / 100;
    ledcWrite(throttle_channel, throttle_out / 4);
  }

    // Send HW data to BT app
  if ( (millis() - timerApp) > timeApp ) {
    // Serial.printf("Data updated 2 - %d\n", V_BAT);
    packTemperatureData();
    packVoltageCurrentData();
    timerApp = millis();
  }

  // DEBUG states
  if ( DEBUG_STATES && (vehicleState != vehicleStatePrevious) ) {
    Serial.printf("State change (3): %d\r\n", vehicleState);
    vehicleStatePrevious = vehicleState;
  }

  // DEBUG plot variables
  if ( ( DEBUG_PLOTTER ) && (millis() - timerPlotter) > timePlotter ) {
    Serial.print("SOC:");
    Serial.print(SOC);
    Serial.print(",");
    Serial.print("Throttle:");
    Serial.print(throttleValue / 10); //
    Serial.print(",");
    Serial.print("ThrottleOut:");
    Serial.print(throttle_out / 10); //
    Serial.print(",");  // throttleCommand
    Serial.print("ThrottleCmd:");
    Serial.print(throttleCommand); //
    Serial.print(","); 
    Serial.print("State:");
    Serial.print(vehicleState*10);
    Serial.print(",");
    Serial.print("Rev:");
    Serial.print( int(reverse)*10 );
    Serial.print(",");
    Serial.print("Fault:");
    Serial.print(hardFault*10);
    Serial.print(",");
    Serial.print("Vbat:");
    Serial.print(V_BAT);
    Serial.print(",");
    Serial.print("VM:");
    Serial.print(V_M);
    Serial.print(",");
    Serial.print("Ibat:");
    Serial.print(I_BAT);
    Serial.print(",");
    Serial.print("Reverse:");
    Serial.print(reverse);
    Serial.print(",");
    Serial.print("For scale:");
    Serial.println(100);
    timerPlotter = millis();
  }

  // delay(50);
}

int read_throttle() {
  int result;
  SUM = SUM - READINGS[INDEX];       // Remove the oldest entry from the sum
  VALUE = analogRead(THROTTLE_PIN);  // Read the next sensor value
  READINGS[INDEX] = VALUE;           // Add the newest reading to the window
  SUM = SUM + VALUE;                 // Add the newest reading to the sum
  INDEX = (INDEX+1) % WINDOW_SIZE;   // Increment the index, and wrap to 0 if it exceeds the window size
  AVERAGED = SUM / WINDOW_SIZE;      // Divide the sum of the window by the window size for the result
  result = ((AVERAGED - throttleMin) * 1000) / throttleRange;
  if (result < 0) { result = 0; }
  return result;
}

int read_vbat() {
  int result;
  SUMvbat = SUMvbat - READINGSvbat[INDEXvbat];       // Remove the oldest entry from the sum
  VALUEvbat = analogRead(VB_ADC_PIN) * 10;  // Read the next sensor value
  // if ( VALUEvbat > AVERAGEDvbat*1.1 ) {
  //   VALUEvbat = AVERAGEDvbat*1.1;
  // } else if ( VALUEvbat < AVERAGEDvbat*0.9 ) {
  //   VALUEvbat = AVERAGEDvbat*0.9;
  // }
  READINGSvbat[INDEXvbat] = VALUEvbat;           // Add the newest reading to the window
  SUMvbat = SUMvbat + VALUEvbat;                 // Add the newest reading to the sum
  INDEXvbat = (INDEXvbat+1) % WINDOW_SIZE_vbat;   // Increment the index, and wrap to 0 if it exceeds the window size
  AVERAGEDvbat = SUMvbat / WINDOW_SIZE_vbat;      // Divide the sum of the window by the window size for the result
  result = AVERAGEDvbat;
  if (result < 0) { result = 0; }
  return result;
}

int set_throttle(int x, int max) {

  // Calculate the error (difference between setpoint and actual value)
  // Limit the change in PWM value to maxChange
  // if ( x > (max*10) ) {
  //   x = max;
  // }
  int deltaPWMValue = constrain(x - previousPWMValue, -maxChangeN, maxChangeP);
  int newPWMValue = previousPWMValue + deltaPWMValue;
  if ( newPWMValue > (max*10) ) {
    newPWMValue = max*10;
  }
  // Update the previous PWM value for the next iteration
  previousPWMValue = newPWMValue;

  return newPWMValue;
}

int read_temp(int x_pin) {
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

int read_voltage(int x_pin) {
  int raw_read = 0;
  int calculated = 0;
  raw_read = analogRead(x_pin);
  calculated = raw_read * xVMADC;
  return calculated;
}

int read_current_bat(int x_pin){
  unsigned long result;
  SUMibat = SUMibat - READINGSibat[INDEXibat];       // Remove the oldest entry from the sum
  VALUEibat = analogRead(x_pin) * 10;  // Read the next sensor value
  // if ( VALUEvbat > AVERAGEDvbat*1.1 ) {
  //   VALUEvbat = AVERAGEDvbat*1.1;
  // } else if ( VALUEvbat < AVERAGEDvbat*0.9 ) {
  //   VALUEvbat = AVERAGEDvbat*0.9;
  // }
  READINGSibat[INDEXibat] = VALUEibat;           // Add the newest reading to the window
  SUMibat = SUMibat + VALUEibat;                 // Add the newest reading to the sum
  INDEXibat = (INDEXibat+1) % WINDOW_SIZE_ibat;   // Increment the index, and wrap to 0 if it exceeds the window size
  AVERAGEDibat = SUMibat / WINDOW_SIZE_ibat;      // Divide the sum of the window by the window size for the result
  
  // Vo = Is * Rs * Rl / 5000
  // Is = Vo * 5000 / ( Rs * Rl )
  int Rs = 2; // mOhm
  int Rl = 220; // kOhm
  unsigned long Vo = (VALUEibat * 100 * 3300) / 4096; // 10-bit: 1024
  result = (Vo * 5000) / ( 2 * Rs * R1 );
  if (result < 0) { result = 0; }

  return result;
}

int read_current_mtr(int x_pin){
  int raw_read = 0;
  int calculated = 0;
  raw_read = analogRead(x_pin);
  calculated = raw_read * xVMADC / 100;
  return raw_read;
}


