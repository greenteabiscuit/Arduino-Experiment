//Blynk で リモートLチカ

#define BLYNK_PRINT Serial
#define BLYNK_USE_DIRECT_CONNECT
#include <BlynkSimpleEsp32_BLE.h>
#include <BLEDevice.h>
#include <BLEServer.h>
char auth[] = "Lq2QhC0nSE79FTHRxu5YxbSTwGkjxLDY";

BlynkTimer timer;

void myTimerEvent()
{
  // You can send any value at any time.
  // Please don't send more that 10 values per second.
  long x , y , z ;
  x = analogRead(33); // Ｘ軸
  y = analogRead(34);
  z = analogRead(35);
  Blynk.virtualWrite(V5, x);
  Blynk.virtualWrite(V6, y);
  Blynk.virtualWrite(V7, z);
  Serial.println(z);
}

void setup(){
  pinMode(21, OUTPUT);
  pinMode(22, OUTPUT);
  pinMode(23, OUTPUT);
  Serial.begin(9600);
  Serial.println("Waiting for connections...");
  Blynk.setDeviceName("L-chika");
  Blynk.begin(auth);
  timer.setInterval(1000L, myTimerEvent);
}

void loop(){
  Blynk.run();
  timer.run();
}
