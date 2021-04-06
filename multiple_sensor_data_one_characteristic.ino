
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include <Wire.h>
#include <SPI.h>

uint8_t accel_sensor_value_x = 0;
uint8_t accel_sensor_value_y = 0;
uint8_t accel_sensor_value_z = 0;

bool deviceConnected = false;

#define NUMBER_OF_SENSORS 4

union multi_sensor_data
{
  struct __attribute__( ( packed ) )
  {
    uint8_t values[NUMBER_OF_SENSORS];
  };
  uint8_t bytes[ NUMBER_OF_SENSORS * sizeof( float ) ];
};

union multi_sensor_data multiSensorData;

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID        "d5875408-fa51-4763-a75d-7d33cecebc31"
#define CHARACTERISTIC_UUID "a4f01d8c-a037-43b6-9050-1876a8c23584"

BLECharacteristic *multiSensorDataCharacteristic;

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
  multiSensorDataCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE
                    );

  // https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.descriptor.gatt.client_characteristic_configuration.xml
  // Create a BLE Descriptor
  multiSensorDataCharacteristic->addDescriptor(new BLE2902());

  // Start the service
  pService->start();

  // Start advertising
  pServer->getAdvertising()->start();
  Serial.println("Waiting a client connection to notify...");
}

void loop() {

  if (deviceConnected) {
    accel_sensor_value_x = analogRead(34); // for universal, x is 32
    accel_sensor_value_y = analogRead(32); // for universal, y is 35
    accel_sensor_value_z = analogRead(35); //for universal, z is 34
    // Serial.printf("*** NOTIFY: %d ***\n", value);
    Serial.printf("*** NOTIFY ***\n");
    char buffer[32];
    // sprintf(buffer, "{\"val\":%d}", value);
    sprintf(buffer, "{\"val_accel_x\":%d}", accel_sensor_value_x);
    Serial.printf(buffer);

    sprintf(buffer, "{\"val_accel_y\":%d}", accel_sensor_value_y);
    Serial.printf(buffer);

    sprintf(buffer, "{\"val_accel_z\":%d}", accel_sensor_value_z);
    Serial.printf(buffer);
    multiSensorData.values[0] = accel_sensor_value_x;
    multiSensorData.values[1] = accel_sensor_value_y;
    multiSensorData.values[2] = accel_sensor_value_z;
    multiSensorDataCharacteristic->setValue( multiSensorData.bytes, sizeof multiSensorData.bytes );
    Serial.println(sizeof multiSensorData.bytes);
    for ( int i = 0; i < sizeof multiSensorData.bytes; i++ )
    {
      Serial.print(multiSensorData.values[i]);
      Serial.print(" ");
    }
    Serial.println();
    
    
    multiSensorDataCharacteristic->notify();

  }
  delay(2000);
}
