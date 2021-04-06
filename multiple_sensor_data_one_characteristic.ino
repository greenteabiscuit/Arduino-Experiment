#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"

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

#define SEALEVELPRESSURE_HPA (1013.25)

#define I2C_SDA 23
#define I2C_SCL 22
TwoWire I2CBME = TwoWire(0);
Adafruit_BME680 bme(&I2CBME); // I2C

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
  Serial.begin(9600);
  I2CBME.begin(I2C_SDA, I2C_SCL, 100000);
  if (!bme.begin()) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
  }
  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms

  // Create the BLE Device
  BLEDevice::init("Up Bluetooth Device");

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
    multiSensorData.values[3] = int(bme.temperature);
    multiSensorData.values[4] = int(bme.humidity);
    multiSensorData.values[5] = int(bme.gas_resistance / 1000.0);
    multiSensorData.values[6] = int(bme.pressure / 1000.0); // hectopascalではなく1000で割っている
    multiSensorDataCharacteristic->setValue( multiSensorData.bytes, sizeof multiSensorData.bytes );
    Serial.println(sizeof multiSensorData.bytes);
    for ( int i = 0; i < sizeof multiSensorData.bytes; i++ )
    {
      Serial.print(multiSensorData.values[i]);
      Serial.print(" ");
    }
    Serial.println();
    
    
    multiSensorDataCharacteristic->notify();
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
  
    Serial.println();

  }
  delay(2000);
}
