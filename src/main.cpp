#include <Arduino.h>
#include <Adafruit_LIS3DH.h>
#include <ESP8266WiFi.h>
#include <time.h>

#include "credentials.h"

// I2C
Adafruit_LIS3DH lis = Adafruit_LIS3DH();

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  Serial.print("Memory heap entering setup() = ");
  size_t startHeap = ESP.getFreeHeap();
  Serial.println(startHeap);
  Serial.print("Fragmentation = ");
  Serial.println(ESP.getHeapFragmentation());

  if (! lis.begin(0x18)) {   // change this to 0x19 for alternative i2c address
    Serial.println("Couldnt start");
    while (1) yield();
  }
  Serial.println("LIS3DH found!");

  lis.setRange(LIS3DH_RANGE_4_G);   // 2, 4, 8 or 16 G!

  Serial.print("Range = "); Serial.print(2 << lis.getRange());
  Serial.println("G");


  Serial.print("Looking for wifi ");
  unsigned long start = millis();
  bool timeout = false;
  WiFi.begin(SSID, PASS);
  while (!timeout && WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    if (millis() - start > 45 * 1000) {
      timeout = true;
    }
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.println("WiFi Connected :-)");
    WiFiClient client;
  } else {
    // not connected, too bad :-(
    Serial.println("Could not connected to WiFi :-(");
  }

}

void loop() {}
