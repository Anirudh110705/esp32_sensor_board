#include <Wire.h>
#include "Adafruit_SHT31.h"

Adafruit_SHT31 sht = Adafruit_SHT31();

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Initialize SHT35
  if (!sht.begin(0x44)) {  // Default I2C address is 0x44
    Serial.println(F("Couldn't find SHT31/SHT35"));
    while (1) delay(1);
  }
}

void loop() {
  // Read temperature and humidity
  float temp = sht.readTemperature();
  float hum = sht.readHumidity();

  // Print results
  Serial.print("Temperature: ");
  Serial.print(temp, 2);
  Serial.print(" °C\tHumidity: ");
  Serial.print(hum, 2);
  Serial.println(" %");

  delay(1000); // Read every 1 second
}
