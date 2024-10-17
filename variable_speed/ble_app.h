#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

#define CONFIG_SERVICE_UUID       "8320b1ab-a552-471c-9ea0-d9fae57fb09b"
#define NAME_CHARACTERISTIC_UUID  "2bf9ea19-d99d-4354-9268-4f398f4492a8"
#define POWER_CONFIG_CHARACTERISTIC_UUID  "9aaf2dba-39ae-461f-a716-924e45fb07e3"
#define TEMPERATURE_CHARACTERISTIC_UUID  "aab09324-034b-4c57-9bb7-8c787bb17025"
#define VOLTAGE_CURRENT_CHARACTERISTIC_UUID  "f7e44533-0cba-4ccd-b425-97fbb8d9940a"
#define CONTROLS_CHARACTERISTIC_UUID "1887a425-dba7-4fff-93d7-88631d0f0261"
#define THROTTLE_CHARACTERISTIC_UUID "0ef3fa0f-f388-4d88-bde4-6a4821853785"
#define STEERING_CHARACTERISTIC_UUID "9d92a0f8-adc7-4df9-bbfb-61d617a5ce7c"

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

#define THROTTLE_IS_REVERSE 0xA
#define THROTTLE_IS_FORWARD 0xF
#define THROTTLE_IS_OFF 0xFF

BLEServer *pServer = NULL; // added
BLECharacteristic *ptestTempCharacteristic;
BLECharacteristic *pVoltageCurrentCharacteristic;
BLECharacteristic *pControlsCharacteristic;
BLECharacteristic *pThrottleCharacteristic;
BLECharacteristic *pSteeringCharacteristic;

bool deviceConnected = false;
bool oldDeviceConnected = false;

char temps[8] = {0};
std::string tempsStr = temps;

char data[8] = {0};
std::string dataStr = data;

char settingsConfig[8] = {0};
std::string settingsConfigStr = settingsConfig;

char throttleData[3] = {0};
std::string throttleDataStr = throttleData;

#define CONTROL_ENABLED_BYTE 0x1
#define CONTROL_DISABLED_BYTE 0xAA

#define ESTOP_BYTE 0
#define OUTOFRANGE_BYTE 1
#define REMOTEENABLE_BYTE 2
#define CONTROLS_LEN 3

static void packControlsData() {
  char data[CONTROLS_LEN+1] = {0};
  data[ESTOP_BYTE] = eStop ? CONTROL_ENABLED_BYTE : CONTROL_DISABLED_BYTE;
  data[OUTOFRANGE_BYTE] = outOfRangeEnable ? CONTROL_ENABLED_BYTE : CONTROL_DISABLED_BYTE;
  data[REMOTEENABLE_BYTE] = remoteEnable ? CONTROL_ENABLED_BYTE : CONTROL_DISABLED_BYTE;
  pControlsCharacteristic->setValue(data);
  pControlsCharacteristic->notify(true);
}

static void unpackControlsData(const char * newData) {
  eStop = newData[ESTOP_BYTE] == CONTROL_ENABLED_BYTE;
  outOfRangeEnable = newData[OUTOFRANGE_BYTE] == CONTROL_ENABLED_BYTE;
  remoteEnable = newData[REMOTEENABLE_BYTE] == CONTROL_ENABLED_BYTE;
  updatedControls = true;
  packControlsData();
}

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

void packThrottleData(int throttle_out, bool reverse) {
  if (remoteEnable == false) {
    int throttleAdjusted = throttle_out / 10;
    if (throttleAdjusted == 0) {
      throttleData[0] = THROTTLE_IS_OFF;
    } else {
      throttleData[0] = reverse ? THROTTLE_IS_REVERSE : THROTTLE_IS_FORWARD;
      throttleData[1] = throttleAdjusted;
    }
    throttleDataStr = throttleData;

    pThrottleCharacteristic->setValue(throttleDataStr);
    pThrottleCharacteristic->notify(true);
  }
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
  data[4] = SOC > 0 ? SOC : 0x1;
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

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      Serial.println("Server Connected");
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      Serial.println("Server disconnected");
    }
};

class CharacteristicCallback: public BLECharacteristicCallbacks {

    // void onConnect(BLEServer* pServer) {
    //   Serial.println("Server Connected");
    // };

    // void onDisconnect(BLEServer* pServer) {
    //   Serial.println("Server disconnected");
    // } 

    void onWrite(BLECharacteristic *pCharacteristic) {
      // Serial.println("Received characteristic update");
      std::string uuid = pCharacteristic->getUUID().toString().c_str();
      const char * value = pCharacteristic->getValue().c_str();

      if (uuid == NAME_CHARACTERISTIC_UUID){
        Serial.println("New name characteristic");
        size_t nameLen = strlen(value);
        if (nameLen >= MIN_NAME_LEN && nameLen <= MAX_NAME_LEN){
          // Directly put in eeprom so we dont worry about passing string around
          preferences.putBytes("name", value, nameLen);
          Serial.printf("Updated name in EEPROM : %s\r\n", value);
        }
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
          
      } else if (uuid == CONTROLS_CHARACTERISTIC_UUID){
        // Serial.println("New controls characteristic");
        const char * newValue = pCharacteristic->getValue().data();
        if (strlen(newValue) >= CONTROLS_LEN) {
          unpackControlsData(newValue);
        } else {
          Serial.println("Error: Can't update controls");
        }
      } else if (uuid == THROTTLE_CHARACTERISTIC_UUID){
        const char * newValue = pCharacteristic->getValue().data();
        if (strlen(newValue) >= 1) {
          int newThrottle = (int) newValue[0];
          if (newThrottle > 100 && newThrottle <= 200){
            newThrottle = 100 - newThrottle;
          } else if (newThrottle < 0 || newThrottle > 200) {
            newThrottle = 0;
          }
          throttleCommand = newThrottle;
          updatedThrottle = true;
        } else {
          // Serial.println("Error: Can't update throttle");
          throttleCommand = 0;
        }
      } else if (uuid == STEERING_CHARACTERISTIC_UUID){
        const char * newValue = pCharacteristic->getValue().data();
        if (strlen(newValue) >= 1) {
          char newSteeringCommand = newValue[0];
          switch (newSteeringCommand) {
            case STEER_STAIGHT_COMMAND: // intentional fall thru
            case STEER_RIGHT_COMMAND: // intentional fall thru
            case STEER_LEFT_COMMAND:
              steeringCommand = newSteeringCommand;
              break;
            default:
              Serial.println("Error: unrecognized steering command");
              steeringCommand = STEER_STAIGHT_COMMAND;
              break;
          }
        } else {
          Serial.println("Error: Can't update steering");
          steeringCommand = STEER_STAIGHT_COMMAND;
        }
        Serial.printf("s: %x\r\n", steeringCommand);
      } else {
        Serial.println("Error: received unknown characteristic");
        throttleCommand = 0;
      }
    }

}; //end of callback

void ble_checkToReconnect() //added
{
  // disconnected so advertise
  if (!deviceConnected && oldDeviceConnected) {
    delay(500); // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising(); // restart advertising
    Serial.println("Disconnected: start advertising");
    oldDeviceConnected = deviceConnected;
  }
  // connected so reset boolean control
  if (deviceConnected && !oldDeviceConnected) {
    // do stuff here on connecting
    Serial.println("Reconnected");
    oldDeviceConnected = deviceConnected;
  }
}

void ble_setup(){
  initializeSettings();
  packSettings();

  BLEDevice::init(deviceNameHW);
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(CONFIG_SERVICE_UUID);

  CharacteristicCallback *pCallbacks = new CharacteristicCallback();

  BLECharacteristic *pNameCharacteristic = pService->createCharacteristic(
                                         NAME_CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );
  pNameCharacteristic->setValue(deviceNameHW);
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

  pControlsCharacteristic = pService->createCharacteristic(
                                         CONTROLS_CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );
  pControlsCharacteristic->setValue(settingsConfigStr);
  pControlsCharacteristic->setCallbacks(pCallbacks);

  pThrottleCharacteristic = pService->createCharacteristic(
                                         THROTTLE_CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );
  pThrottleCharacteristic->setValue(throttleCommand);
  pThrottleCharacteristic->setCallbacks(pCallbacks);

  pSteeringCharacteristic = pService->createCharacteristic(
                                         STEERING_CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );
  pSteeringCharacteristic->setValue(throttleCommand);
  pSteeringCharacteristic->setCallbacks(pCallbacks);
  
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