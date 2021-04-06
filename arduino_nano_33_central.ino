/*
  This example creates a BLE central that scans for a peripheral with a service containing a multi value characteristic.

  The circuit:
  - Arduino Nano 33 BLE or Arduino Nano 33 IoT board.

  This example code is in the public domain.
*/

#include <ArduinoBLE.h>

//----------------------------------------------------------------------------------------------------------------------
// BLE UUIDs
//----------------------------------------------------------------------------------------------------------------------

#define BLE_UUID_SENSOR_DATA_SERVICE              "2BEEF31A-B10D-271C-C9EA-35D865C1F48A"
#define BLE_UUID_MULTI_SENSOR_DATA                "4664E7A1-5A13-BFFF-4636-7D0A4B16496C"

#define NUMBER_OF_SENSORS 4

union multi_sensor_data
{
  struct __attribute__( ( packed ) )
  {
    float values[NUMBER_OF_SENSORS];
  };
  uint8_t bytes[ NUMBER_OF_SENSORS * sizeof( float ) ];
};

union multi_sensor_data multiSensorData;


void setup()
{
  Serial.begin( 9600 );
  while ( !Serial );

  BLE.begin();

  BLE.scanForUuid( BLE_UUID_SENSOR_DATA_SERVICE );
}


void loop()
{
#define UPDATE_INTERVALL 10
  static long previousMillis = 0;

  unsigned long currentMillis = millis();
  if ( currentMillis - previousMillis > UPDATE_INTERVALL )
  {
    previousMillis = currentMillis;

    BLEDevice peripheral = BLE.available();

    if ( peripheral )
    {
      if ( peripheral.localName() != "Arduino Nano 33 BLE" )
      {
        return;
      }

      BLE.stopScan();

      explorePeripheral( peripheral );

      BLE.scanForUuid( BLE_UUID_SENSOR_DATA_SERVICE );
    }
  }
}


bool explorePeripheral( BLEDevice peripheral )
{
  if ( !peripheral.connect() )
  {
    return false;
  }
  Serial.println( "BLE connected" );

  if ( !peripheral.discoverAttributes() )
  {
    peripheral.disconnect();
    return false;
  }
  Serial.println( "BLE attributes discovered" );

  BLECharacteristic multiSensorDataCharacteristic = peripheral.characteristic( BLE_UUID_MULTI_SENSOR_DATA );
  if ( !multiSensorDataCharacteristic )
  {
    peripheral.disconnect();
    return false;
  }
  Serial.println( "BLE characteristic found" );

  if ( !multiSensorDataCharacteristic.canSubscribe() )
  {
    peripheral.disconnect();
    return false;
  }
  Serial.println( "BLE characteristic can subscribe" );

  if ( !multiSensorDataCharacteristic.subscribe() )
  {
    peripheral.disconnect();
    return false;
  }
  Serial.println( "BLE characteristic subscribed" );

  while ( 1 ) // need to add logic to leave
  {
#define BLE_POLL_INTERVALL 5
    static long previousMillis = 0;
    unsigned long currentMillis = millis();
    if ( currentMillis - previousMillis > BLE_POLL_INTERVALL )
    {
      BLE.poll();

      if ( multiSensorDataCharacteristic.valueUpdated() )
      {
        Serial.println( "BLE new data" );

        multiSensorDataCharacteristic.readValue( multiSensorData.bytes, sizeof multiSensorData.bytes );
        for ( int i = 0; i < NUMBER_OF_SENSORS; i++ )
        {
          Serial.print( "Sensor (" );
          Serial.print( i );
          Serial.print( "): " );
          Serial.println( multiSensorData.values[i] );
        }
      }
    }
  }

  peripheral.disconnect();
  return true;
}
