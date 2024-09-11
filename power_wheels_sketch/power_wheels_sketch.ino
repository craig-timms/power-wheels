/*
    Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleServer.cpp
    Ported to Arduino ESP32 by Evandro Copercini
    updates by chegewara
*/

// Craig - reference these variables in the loop()
int batteryLevel = 20;
bool reverseDirection = false;
bool reverseSteering = false;
int maxPower = 100;
int reversePowerMax = 20;
int throttleMaxPowerPos = 100;
int throttleVsPowerMap = 50;
// END
bool updatedSettings = false;

#include "ble_app.h"

void setup() {
  Serial.begin(115200);
  Serial.println("Starting BLE work");

  ble_setup();
  Serial.println("Setup complete. Listening for BLE updates...");
  
  // delay(1000);
}

void loop() {
  // put your main code here, to run repeatedly:
  // Serial.println("In loop...");
  // delay(200);
  if (updatedSettings == true) {
    Serial.println("Craig - process the new settings here");
    // ADD CODE HERE
    Serial.printf("New Battery Voltage: %d\r\n", batteryLevel); // example of referencing the updated settings
    //
    //
    //
    //
    updatedSettings = false;
  }
}