#define MONITOR_SERIAL Serial
#define RADAR_SERIAL Serial1
#define RADAR_RX_PIN 16
#define RADAR_TX_PIN 17

#include <WiFi.h>
#include <Wire.h>
#include "Adafruit_SHT31.h"
#include <math.h>
#include <ld2410.h>

// ---------------- WiFi Settings ----------------
const char* ssid = "DEMETER";
const char* password = "Demeter123!";
WiFiServer server(3333);
WiFiClient client;

// ---------------- Sensor Pins ------------------
const int soundSensor2Pin = 35;  // MAX9814 analog output

// ---------------- ADC / Timing -----------------
const float VREF = 3.3f;       
const int ADC_RES = 4095;      
const int SAMPLE_WINDOW_MS = 50;
const int SAMPLE_RATE = 2000;  
const float DB_SMOOTH_ALPHA = 0.5f;

// Calibration
float refVrms = 0.02f; 
float refDb   = 40.0f; 

// ---------------- SHT & LD2410 -----------------
Adafruit_SHT31 sht = Adafruit_SHT31();
ld2410 radar;
uint32_t lastReading = 0;

// LD2410 buffer
int movingBuffer[5];
int bufferIndex = 0;
bool bufferFilled = 0;
int detector = 0;  // 1 = detected, 0 = not detected

// ---------------- Helper Functions ----------------
float measureVrms(int pin, int windowMs) {
  unsigned long start = millis();
  unsigned long sampleIntervalUs = 1000000UL / SAMPLE_RATE;
  unsigned long nextMicros = micros();
  unsigned long count = 0;
  double sum = 0.0;
  double sumsq = 0.0;

  while ((int)(millis() - start) < windowMs) {
    int adc = analogRead(pin);
    double v = (adc / (double)ADC_RES) * VREF;
    sum += v;
    sumsq += v * v;
    count++;
    nextMicros += sampleIntervalUs;
    long wait = (long)(nextMicros - micros());
    if (wait > 0) delayMicroseconds(wait);
  }
  if (count == 0) return 0.0f;
  double mean = sum / (double)count;
  double meanSq = sumsq / (double)count;
  double vrms = sqrt(fmax(0.0, meanSq - mean * mean));
  return (float)vrms;
}

float smoothDbIIR(float db) {
  static float s = -1000.0f;
  if (s < -500.0f) s = db;
  s = DB_SMOOTH_ALPHA * db + (1.0f - DB_SMOOTH_ALPHA) * s;
  return s;
}

// ---------------- Setup ----------------
void setup() {
  Serial.begin(115200);
  delay(1000);

  // ---- SHT35 ----
  Wire.begin();
  if (!sht.begin(0x44)) {
    Serial.println(F("Couldn't find SHT31/SHT35"));
    while (1) delay(1);
  }
  analogReadResolution(12);

  // ---- LD2410 ----
  RADAR_SERIAL.begin(256000, SERIAL_8N1, 16, 17); // RX=16, TX=17
  if (radar.begin(RADAR_SERIAL)) {
    Serial.println("LD2410 connected!");
  } else {
    Serial.println("LD2410 not connected!");
  }

  // ---- Wi-Fi ----
  Serial.println("\nConnecting to WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");
  Serial.print("ESP32 IP address: ");
  Serial.println(WiFi.localIP());

  server.begin();
  Serial.println("TCP server started on port 3333");
}

// ---------------- Loop ----------------
void loop() {
  // Accept new TCP client
  if (!client || !client.connected()) {
    client = server.available();
    if (client) {
      Serial.println("New TCP client connected");
    }
  }

    // ---- LD2410 ----
  radar.read();
  if (radar.isConnected() && millis() - lastReading > 200) 
  {
    lastReading = millis();

    int movingDist = 0;
    if (radar.presenceDetected() && radar.movingTargetDetected()) {
      movingDist = radar.movingTargetDistance();
    }

    // store in buffer
    movingBuffer[bufferIndex] = movingDist;
    bufferIndex++;

    if (bufferIndex >= 10) 
    {
      bufferIndex = 0;
      bufferFilled = true;

      int zeroCount = 0;
      for (int i = 0; i < 10; i++)
      {
        if (movingBuffer[i] == 0) zeroCount++;
      }
      if(zeroCount>4)
      {
        detector=0;
      Serial.print("Person NOT detected\t");  
      } 
      else
      {
        detector =1;
        Serial.print("Person detected\t");
        Serial.print(radar.stationaryTargetDistance());
        Serial.print("\t");
      }
    }
  }

  // ---- Read SHT35 ----
  float temp = sht.readTemperature();
  float hum = sht.readHumidity();

  // ---- Sound RMS ----
  float vrms = measureVrms(soundSensor2Pin, SAMPLE_WINDOW_MS);
  float dB = -999.0f;
  float gainFactor = 1.3f;
  float calibrationOffset = 10;
  if (vrms > 0 && refVrms > 0) {
    dB = refDb + gainFactor * (20.0f * log10f(vrms / refVrms)) - calibrationOffset;
  }
  float dB_smooth = smoothDbIIR(dB);
  if (dB_smooth < 20.0f) dB_smooth = 20.0f;
  if (dB_smooth > 110.0f) dB_smooth = 110.0f;



  // ---- Output ----
  
  /*if (detector == 1) {
    Serial.print("Person detected\t");
    Serial.print(radar.stationaryTargetDistance());
    Serial.print("\t");
  } else {
    Serial.print("Person NOT detected\t");
  }*/

  Serial.print(temp, 2); Serial.print("\t");
  Serial.print(hum, 2); Serial.print("\t");
  Serial.print(vrms * 1000.0f, 3); Serial.print("\t");
  Serial.println(dB_smooth, 2);

  if (client && client.connected()) {
    client.printf("%d %.2f %.2f %.3f %.2f\n", detector, temp, hum, vrms * 1000.0f, dB_smooth);
    // Format: DETECTOR TEMP HUM VRMS_mV DB
  }
}
