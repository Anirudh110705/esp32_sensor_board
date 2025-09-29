#if defined(ESP32)
  #ifdef ESP_IDF_VERSION_MAJOR // IDF 4+
    #if CONFIG_IDF_TARGET_ESP32 // ESP32/PICO-D4
      #define MONITOR_SERIAL Serial
      #define RADAR_SERIAL Serial1
      #define RADAR_RX_PIN 16
      #define RADAR_TX_PIN 17
    #elif CONFIG_IDF_TARGET_ESP32S2
      #define MONITOR_SERIAL Serial
      #define RADAR_SERIAL Serial1
      #define RADAR_RX_PIN 9
      #define RADAR_TX_PIN 8
    #elif CONFIG_IDF_TARGET_ESP32C3
      #define MONITOR_SERIAL Serial
      #define RADAR_SERIAL Serial1
      #define RADAR_RX_PIN 4
      #define RADAR_TX_PIN 5
    #else 
      #error Target CONFIG_IDF_TARGET is not supported
    #endif
  #else // ESP32 Before IDF 4.0
    #define MONITOR_SERIAL Serial
    #define RADAR_SERIAL Serial1
    #define RADAR_RX_PIN 32
    #define RADAR_TX_PIN 33
  #endif
#elif defined(__AVR_ATmega32U4__)
  #define MONITOR_SERIAL Serial
  #define RADAR_SERIAL Serial1
  #define RADAR_RX_PIN 0
  #define RADAR_TX_PIN 1
#endif

#include <ld2410.h>

ld2410 radar;

uint32_t lastReading = 0;

// buffer for last 10 moving values
int movingBuffer[10];
int bufferIndex = 0;
bool bufferFilled = false;
int detector;
void setup(void)
{
  MONITOR_SERIAL.begin(115200);
  #if defined(ESP32)
    RADAR_SERIAL.begin(256000, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN);
  #elif defined(__AVR_ATmega32U4__)
    RADAR_SERIAL.begin(256000);
  #endif
  delay(500);

  if(radar.begin(RADAR_SERIAL))
  {
    MONITOR_SERIAL.println("LD2410 connected!");
  }
  else
  {
    MONITOR_SERIAL.println("LD2410 not connected!");
  }
}

void loop()
{
  radar.read();

  if(radar.isConnected() && millis() - lastReading > 200)  // every 200 ms
  {
    lastReading = millis();

    int movingDist = 0;
    if(radar.presenceDetected() && radar.movingTargetDetected())
    {
      movingDist = radar.movingTargetDistance();
    }

    // store in buffer
    movingBuffer[bufferIndex] = movingDist;
    bufferIndex++;

    if(bufferIndex >= 10)  // once every 10 readings
    {
      bufferIndex = 0;
      bufferFilled = true;

      // count how many zeros in last 10 readings
      int zeroCount = 0;
      for(int i=0; i<10; i++)
      {
        if(movingBuffer[i] == 0) zeroCount++;
      }

      if(zeroCount > 4)
        {
          detector = 0;
          MONITOR_SERIAL.print("Person NOT detected ");
        Serial.println(detector);
        }
      else
      {
        detector =1;
        MONITOR_SERIAL.print("Person detected");
        Serial.println(detector);
      }
    }

    // optional: still print raw values for debugging
    MONITOR_SERIAL.print("Moving:");
    MONITOR_SERIAL.println(movingDist);
  }
}
