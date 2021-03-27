#include <Nefry.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Wire.h>

BLECharacteristic *pCharacteristic_X;
BLECharacteristic *pCharacteristic_Y;
BLECharacteristic *pCharacteristic_Z;
bool deviceConnected = false;
uint8_t value = 0;
uint8_t accel_sensor_value_x = 0;
uint8_t accel_sensor_value_y = 0;
uint8_t accel_sensor_value_z = 0;

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID        "d5875408-fa51-4763-a75d-7d33cecebc31"
#define CHARACTERISTIC_UUID_ACCEL_X "a4f01d8c-a037-43b6-9050-1876a8c23584"
#define CHARACTERISTIC_UUID_ACCEL_Y "a570dd86-0218-4848-a9e5-ada20d43fb1d"
#define CHARACTERISTIC_UUID_ACCEL_Z "1b5cbd6e-85b3-4528-ab1b-cd5fb8d85380"

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

void setup() {
  Serial.begin(115200);

  // Create the BLE Device
  BLEDevice::init("NefryBT");

  // Create the BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pCharacteristic_X = pService->createCharacteristic(
                      CHARACTERISTIC_UUID_ACCEL_X,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE
                    );
  // Create a BLE Characteristic for accelerator y
  pCharacteristic_Y = pService->createCharacteristic(
                      CHARACTERISTIC_UUID_ACCEL_Y,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE
                    );

  // Create a BLE Characteristic for accelerator z
  pCharacteristic_Z = pService->createCharacteristic(
                      CHARACTERISTIC_UUID_ACCEL_Z,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE
                    );

  // https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.descriptor.gatt.client_characteristic_configuration.xml
  // Create a BLE Descriptor
  pCharacteristic_X->addDescriptor(new BLE2902());
  pCharacteristic_Y->addDescriptor(new BLE2902());
  pCharacteristic_Z->addDescriptor(new BLE2902());

  // Start the service
  pService->start();

  // Start advertising
  pServer->getAdvertising()->start();
  Serial.println("Waiting a client connection to notify...");
}

void loop() {

  if (deviceConnected) {
    accel_sensor_value_x = analogRead(32);
    accel_sensor_value_y = analogRead(35);
    accel_sensor_value_z = analogRead(34);
    // Serial.printf("*** NOTIFY: %d ***\n", value);
    Serial.printf("*** NOTIFY: %d ***\n", accel_sensor_value_x);
    char buffer[32];
    // sprintf(buffer, "{\"val\":%d}", value);
    sprintf(buffer, "{\"val_accel_x\":%d}", accel_sensor_value_x);
    Serial.printf(buffer);
    pCharacteristic_X->setValue(buffer);
    pCharacteristic_X->notify();

    sprintf(buffer, "{\"val_accel_y\":%d}", accel_sensor_value_y);
    Serial.printf(buffer);
    pCharacteristic_Y->setValue(buffer);
    pCharacteristic_Y->notify();

    sprintf(buffer, "{\"val_accel_z\":%d}", accel_sensor_value_z);
    Serial.printf(buffer);
    pCharacteristic_Z->setValue(buffer);
    pCharacteristic_Z->notify();
    //pCharacteristic->indicate();
    // value++;
  }
  delay(2000);
}
