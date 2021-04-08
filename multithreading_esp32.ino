/*
 *  Copyright(C) 2018 by Yukiya Ishioka
 *  https://qiita.com/Yukiya_Ishioka/items/ae21b4a1aecf69a885f2
 */

#include <freertos/FreeRTOS.h>    /* FreeRTOSを用いるためのヘッダファイル */
#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"

MAX30105 particleSensor;

#define MAX_BRIGHTNESS 255

#define  LED1PIN   22    /* LED1のポート番号 */
#define  LED2PIN   23    /* LED2のポート番号 */

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
//Arduino Uno doesn't have enough SRAM to store 100 samples of IR led data and red led data in 32-bit format
//To solve this problem, 16-bit MSB of the sampled data will be truncated. Samples become 16-bit data.
uint16_t irBuffer[100]; //infrared LED sensor data
uint16_t redBuffer[100];  //red LED sensor data
#else
uint32_t irBuffer[100]; //infrared LED sensor data
uint32_t redBuffer[100];  //red LED sensor data
#endif

int32_t bufferLength; //data length
int32_t spo2; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate; //heart rate value
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid

byte pulseLED = 11; //Must be on PWM pin
byte readLED = 13; //Blinks with each data read

uint8_t accel_sensor_value_x = 0;
uint8_t accel_sensor_value_y = 0;
uint8_t accel_sensor_value_z = 0;

void  task1( void *param )
{
  Serial.println("byue");
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
      vTaskDelay(500);
  }
}

void  task2( void *param )
{
  while( 1 ) {
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
    vTaskDelay(500);
  }
}

void setup()
{
  Serial.begin(115200); // initialize serial communication at 115200 bits per second:

  pinMode(pulseLED, OUTPUT);
  pinMode(readLED, OUTPUT);

  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println(F("MAX30105 was not found. Please check wiring/power."));
    while (1);
  }

  Serial.println(F("Attach sensor to finger with rubber band. Press any key to start conversion"));
  while (Serial.available() == 0) ; //wait until user presses a key
  Serial.read();

  byte ledBrightness = 60; //Options: 0=Off to 255=50mA
  byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 4096; //Options: 2048, 4096, 8192, 16384

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings

  /* create task */
  xTaskCreatePinnedToCore( task1,   /* タスクの入口となる関数名 */
                           "TASK1", /* タスクの名称 */
                           0x800,   /* スタックサイズ */
                           NULL,    /* パラメータのポインタ */
                           1,       /* プライオリティ */
                           NULL,    /* ハンドル構造体のポインタ */
                           0 );     /* 割り当てるコア (0/1) */

  xTaskCreatePinnedToCore( task2,
                           "TASK2",
                           0x800,
                           NULL,
                           2,
                           NULL,
                           0 );
}

void loop()
{
  Serial.print("loop()\n");
  vTaskDelay(1000);
}
