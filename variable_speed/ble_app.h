#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

#define CONFIG_SERVICE_UUID       "8320b1ab-a552-471c-9ea0-d9fae57fb09b"
#define NAME_CHARACTERISTIC_UUID  "2bf9ea19-d99d-4354-9268-4f398f4492a8"
#define POWER_CONFIG_CHARACTERISTIC_UUID  "9aaf2dba-39ae-461f-a716-924e45fb07e3"
#define TEMPERATURE_CHARACTERISTIC_UUID  "aab09324-034b-4c57-9bb7-8c787bb17025"
#define VOLTAGE_CURRENT_CHARACTERISTIC_UUID  "f7e44533-0cba-4ccd-b425-97fbb8d9940a"

#define BATTERY_VOLTAGE_INDEX 0
#define DIRECTION_SETTING_INDEX 1
#define STEERING_SETTING_INDEX 2
#define MAX_POWER_INDEX 3
#define REVERSE_POWER_INDEX 4
#define THROTTLE_MAX_POWER_POS_INDEX 5
#define THROTTLE_VS_POWER_MAP_INDEX 6
#define POWER_CONFIG_SETTINGS_LENGTH 7

#define ENABLE_REVERSE_BYTE 0xAA
#define NORMAL_CONFIG_BYTE 0xD

BLECharacteristic *ptestTempCharacteristic;
BLECharacteristic *pVoltageCurrentCharacteristic;

char temps[8] = {0};
std::string tempsStr = temps;

char data[8] = {0};
std::string dataStr = data;

char settingsConfig[8] = {0};
std::string settingsConfigStr = settingsConfig;

void packTemperatureData() {  
  temps[0] = Temp_1 > 0 ? (char)Temp_1 : 0xFF;
  temps[1] = Temp_2 > 0 ? (char)Temp_2 : 0xFF;
  temps[2] = Temp_3 > 0 ? (char)Temp_3 : 0xFF;
  temps[3] = Temp_4 > 0 ? (char)Temp_4 : 0xFF;
  temps[4] = Temp_5 > 0 ? (char)Temp_5 : 0xFF;
  tempsStr = temps;

  ptestTempCharacteristic->setValue(tempsStr);
  ptestTempCharacteristic->notify(true);
}

void packVoltageCurrentData() {
  char v_upper = (V_BAT >> 8) & 0xFF;
  char v_lower = V_BAT & 0xFF;
  char i_upper = (I_BAT >> 8) & 0xFF;
  char i_lower = I_BAT & 0xFF;
  data[0] = v_lower > 0 ? v_lower : 0x1;
  data[1] = v_upper > 0 ? v_upper : 0x1;
  data[2] = i_lower > 0 ? i_lower : 0x1;
  data[3] = i_upper > 0 ? i_upper : 0x1;
  Serial.printf("Voltage Data: %x", (int)data);
  dataStr = data;
  pVoltageCurrentCharacteristic->setValue(dataStr);
  pVoltageCurrentCharacteristic->notify(true);
}

static void initializeSettings() {
  reverseDirection = reverseDirectionHW;
  reverseSteering = reverseSteeringHW;
  maxPower = maxPowerHW;
  reversePowerMax = reversePowerMaxHW;
  throttleMaxPowerPos = throttleMaxPowerPosHW;
  throttleVsPowerMap = throttleVsPowerMap;
}

static void packSettings() {
  settingsConfig[BATTERY_VOLTAGE_INDEX] = (char)batteryLevel;
  settingsConfig[DIRECTION_SETTING_INDEX] = reverseDirection ? ENABLE_REVERSE_BYTE : NORMAL_CONFIG_BYTE;
  settingsConfig[STEERING_SETTING_INDEX] = reverseSteering ? ENABLE_REVERSE_BYTE : NORMAL_CONFIG_BYTE;
  settingsConfig[MAX_POWER_INDEX] = (char)maxPower;
  settingsConfig[REVERSE_POWER_INDEX] = (char)reversePowerMax;
  settingsConfig[THROTTLE_MAX_POWER_POS_INDEX] = (char)throttleMaxPowerPos;
  settingsConfig[THROTTLE_VS_POWER_MAP_INDEX] = (char)throttleVsPowerMap;
  settingsConfigStr = settingsConfig;
}

static void unpackSettings(const char * s){
  int newBatteryLevel = (int) (s[BATTERY_VOLTAGE_INDEX]);
  if (newBatteryLevel >= 0 && newBatteryLevel <= 36) {
    batteryLevel = newBatteryLevel;
  }
  reverseDirection = s[DIRECTION_SETTING_INDEX] == ENABLE_REVERSE_BYTE;
  reverseSteering = s[STEERING_SETTING_INDEX] == ENABLE_REVERSE_BYTE;
  
  int newMaxPower = (int) (s[MAX_POWER_INDEX]);
  if (newMaxPower > 0 && newMaxPower <= 100) {
    maxPower = newMaxPower;
  }

  int newReversePowerMax = (int) (s[REVERSE_POWER_INDEX]);
  if (newReversePowerMax > 0 && newReversePowerMax <= 100) {
    reversePowerMax = newReversePowerMax;
  }

  int newThrottleMaxPower = (int) (s[THROTTLE_MAX_POWER_POS_INDEX]);
  if (newThrottleMaxPower > 0 && newThrottleMaxPower <= 100) {
    throttleMaxPowerPos = newThrottleMaxPower;
  }

  int newThrottleVsPower = (int) (s[THROTTLE_VS_POWER_MAP_INDEX]);
  if (newThrottleVsPower > 0 && newThrottleVsPower <= 100) {
    throttleVsPowerMap = newThrottleVsPower;
  }
  updatedSettings = true;
  packSettings();
}

class CharacteristicCallback: public BLECharacteristicCallbacks {

    void onConnect(BLEServer* pServer) {
      Serial.println("Server Connected");
    };

    void onDisconnect(BLEServer* pServer) {
      Serial.println("Server disconnected");
    } 

    void onWrite(BLECharacteristic *pCharacteristic) {
      Serial.println("Received characteristic update");
      std::string uuid = pCharacteristic->getUUID().toString().c_str();
      const char * value = pCharacteristic->getValue().c_str();

      if (uuid == NAME_CHARACTERISTIC_UUID){
        Serial.println("New name characteristic");
        Serial.printf("Value: %s\r\n", value);
        pCharacteristic->setValue(value);
        pCharacteristic->notify(true);
      } else if (uuid == POWER_CONFIG_CHARACTERISTIC_UUID){
        Serial.println("New power config characteristic");
        const char * newValue = pCharacteristic->getValue().data();
        if (strlen(newValue) >= POWER_CONFIG_SETTINGS_LENGTH) {
          unpackSettings(newValue);
        } else {
          Serial.println("Error: Can't update settings");
        }
        pCharacteristic->setValue(settingsConfigStr);
        pCharacteristic->notify(true);
          
      } else {
        Serial.println("Error: received unknown characteristic");
      }
    }

}; //end of callback

void ble_setup(){
  initializeSettings();
  packSettings();

  BLEDevice::init("Power Wheels");
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(CONFIG_SERVICE_UUID);

  CharacteristicCallback *pCallbacks = new CharacteristicCallback();

  BLECharacteristic *pNameCharacteristic = pService->createCharacteristic(
                                         NAME_CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );
  pNameCharacteristic->setValue("Power Wheels Default Device");
  pNameCharacteristic->setCallbacks(pCallbacks);

  BLECharacteristic *pBatteryVoltageCharacteristic = pService->createCharacteristic(
                                         POWER_CONFIG_CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );
  pBatteryVoltageCharacteristic->setValue(settingsConfigStr);
  pBatteryVoltageCharacteristic->setCallbacks(pCallbacks);

  ptestTempCharacteristic = pService->createCharacteristic(
                                         TEMPERATURE_CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ
                                       );
  ptestTempCharacteristic->setValue(tempsStr);

  pVoltageCurrentCharacteristic = pService->createCharacteristic(
                                         VOLTAGE_CURRENT_CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ
                                       );
  pVoltageCurrentCharacteristic->setValue(dataStr);
  
  pService->start();
  // BLEAdvertising *pAdvertising = pServer->getAdvertising();  // this still is working for backward compatibility
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(CONFIG_SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  // pAdvertising->start();  
}