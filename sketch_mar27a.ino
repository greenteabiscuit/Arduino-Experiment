#include <Nefry.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// For BME680
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"

TwoWire I2CBME = TwoWire(0);
Adafruit_BME680 bme; // I2C


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

#define I2C_SDA 18
#define I2C_SCL 19
#define SEALEVELPRESSURE_HPA (1013.25)

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
  if (!I2CBME.begin(I2C_SDA, I2C_SCL, 100000)) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
  }

  bool status;

  // default settings
  // (you can also pass in a Wire library object like &Wire2)
  status = bme.begin(0x76, &I2CBME);  
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
  }

  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms

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
    Serial.print("Temperature = ");
    Serial.print(bme.temperature);
    Serial.println(" *C");
  
    Serial.print("Pressure = ");
    Serial.print(bme.pressure / 100.0);
    Serial.println(" hPa");
  
    Serial.print("Humidity = ");
    Serial.print(bme.humidity);
    Serial.println(" %");
  
    Serial.print("Gas = ");
    Serial.print(bme.gas_resistance / 1000.0);
    Serial.println(" KOhms");
  
    Serial.print("Approx. Altitude = ");
    Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.println(" m");
  }
  delay(2000);
}
