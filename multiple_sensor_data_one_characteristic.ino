#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_MLX90614.h>

TwoWire I2CBME = TwoWire(0);
Adafruit_BME680 bme(&I2CBME); // I2C

TwoWire I2CMLX = TwoWire(1);
Adafruit_MLX90614 mlx = Adafruit_MLX90614(0x5A, &I2CMLX);

TwoWire I2CMAX30105 = TwoWire(2);
MAX30105 particleSensor;

int32_t bufferLength; //data length
int32_t spo2; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate; //heart rate value
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid

uint8_t accel_sensor_value_x = 0;
uint8_t accel_sensor_value_y = 0;
uint8_t accel_sensor_value_z = 0;

float ambientTemp = 0;
float objectTemp = 0;

bool deviceConnected = false;

#define SEALEVELPRESSURE_HPA (1013.25)

#define I2C_BME680_SDA 23 // v216のBME680のSDAは23
#define I2C_BME680_SCL 17 // v216のBME680のSCLは17
#define I2C_SDA 21 // v216のBME680のSDAは23
#define I2C_SCL 22 // v216のBME680のSCLは17
#define I2C_MAX30002_SCL 18
#define I2C_MAX30002_SDA 19

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

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
  //Arduino Uno doesn't have enough SRAM to store 100 samples of IR led data and red led data in 32-bit format
  //To solve this problem, 16-bit MSB of the sampled data will be truncated. Samples become 16-bit data.
  uint16_t irBuffer[100]; //infrared LED sensor data
  uint16_t redBuffer[100];  //red LED sensor data
#else
  uint32_t irBuffer[100]; //infrared LED sensor data
  uint32_t redBuffer[100];  //red LED sensor data
#endif

byte pulseLED = 11; //Must be on PWM pin
byte readLED = 13; //Blinks with each data read

BLECharacteristic *multiSensorDataCharacteristic;

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

void  task_heart_rate( void *param )
{
  Serial.println("heartrate");
  if (deviceConnected) {
    bufferLength = 100; //buffer length of 100 stores 4 seconds of samples running at 25sps
  
    //read the first 100 samples, and determine the signal range
    for (byte i = 0 ; i < bufferLength ; i++)
    {
      while (particleSensor.available() == false) {//do we have new data?
        Serial.println("no data");
        particleSensor.check(); //Check the sensor for new data
      }
      redBuffer[i] = particleSensor.getRed();
      irBuffer[i] = particleSensor.getIR();
      particleSensor.nextSample(); //We're finished with this sample so move to next sample
  
      Serial.print(F("red="));
      Serial.print(redBuffer[i], DEC);
      Serial.print(F(", ir="));
      Serial.println(irBuffer[i], DEC);
    }
  
    //calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
    
    while( 1 ) {
        //dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
        for (byte i = 25; i < 100; i++)
        {
          redBuffer[i - 25] = redBuffer[i];
          irBuffer[i - 25] = irBuffer[i];
        }
    
        //take 25 sets of samples before calculating the heart rate.
        for (byte i = 75; i < 100; i++)
        {
          while (particleSensor.available() == false) //do we have new data?
            particleSensor.check(); //Check the sensor for new data
    
          digitalWrite(readLED, !digitalRead(readLED)); //Blink onboard LED with every data read
    
          redBuffer[i] = particleSensor.getRed();
          irBuffer[i] = particleSensor.getIR();
          particleSensor.nextSample(); //We're finished with this sample so move to next sample
    
          //send samples and calculation result to terminal program through UART
          Serial.print(F("red="));
          Serial.print(redBuffer[i], DEC);
          Serial.print(F(", ir="));
          Serial.print(irBuffer[i], DEC);
    
          Serial.print(F(", HR="));
          Serial.print(heartRate, DEC);
    
          Serial.print(F(", HRvalid="));
          Serial.print(validHeartRate, DEC);
    
          Serial.print(F(", SPO2="));
          Serial.print(spo2, DEC);
    
          Serial.print(F(", SPO2Valid="));
          Serial.println(validSPO2, DEC);
        }
    
        //After gathering 25 new samples recalculate HR and SP02
        maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
        vTaskDelay(2000);
    }

  }
}

void setup() {
  Serial.begin(115200);
  // Initialize heart ratesensor
  I2CMAX30105.begin(I2C_MAX30002_SDA, I2C_MAX30002_SCL, 0x57);
  if (!particleSensor.begin(I2CMAX30105, 50000, 0x57))
  {
    Serial.println(F("MAX30105 was not found. Please check wiring/power."));
  }
  I2CBME.begin(I2C_BME680_SDA, I2C_BME680_SCL, 0x77);
  if (!bme.begin(0x77, true)) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
  }
  I2CMLX.begin(I2C_SDA, I2C_SCL, 0x5A);

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
  if (!mlx.begin()) {
    Serial.println("unable to start mlx");  
  }

  byte ledBrightness = 60; //Options: 0=Off to 255=50mA
  byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 4096; //Options: 2048, 4096, 8192, 16384

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings
}

void loop() {

  if (deviceConnected) {
    ambientTemp = mlx.readAmbientTempC();
    objectTemp = mlx.readObjectTempC();
    Serial.println(ambientTemp);
    Serial.println(objectTemp);
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
    multiSensorData.values[7] = int(ambientTemp);
    multiSensorData.values[8] = int(objectTemp); 
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
