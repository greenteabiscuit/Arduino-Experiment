#include <WiFi.h>

const char SSID[] = "ssid";
const char PASSWORD[] = "pw";

WiFiServer server(80);

void setup() {
  Serial.begin(115200);
  while (!Serial);

  WiFi.begin(SSID, PASSWORD);
  Serial.print("WiFi connecting");

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(100);
  }

  Serial.println(" connected");

  server.begin();

  Serial.print("HTTP Server: http://");
  Serial.print(WiFi.localIP());
  Serial.println("/");
}

void loop() {
  WiFiClient client = server.available();
  int rawX = analogRead(34);
  Serial.println(rawX);
  if (client) {
    client.println("Hello World!");
    client.println(rawX);
    client.stop();
  }
}
