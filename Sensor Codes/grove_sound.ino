// ===================== Grove Sound Sensor Plot =====================
// ESP32 + Seeed Grove Sound Sensor (analog OUT)
// Plots raw ADC (mV) on Serial Plotter
// Includes fixed REF lines to prevent autoscaling

const int GROVE_PIN = 35;  // Analog pin connected to Grove Sound Sensor

const float VREF = 3.3f;
const int ADC_RES = 4095;
const int LOOP_DELAY_MS = 200;

// Plot bounds
const float PLOT_MIN = 0.0f;
const float PLOT_MAX = 1200.0f;  // expected max in mV

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);

  Serial.println("Grove_mV REF_LOW REF_HIGH");  // For Serial Plotter
}

void loop() {
  int rawADC = analogRead(GROVE_PIN);
  float voltage = (rawADC / (float)ADC_RES) * VREF;
  float mV = voltage * 1000.0f;

  // Clamp
  if (mV < PLOT_MIN) mV = PLOT_MIN;
  if (mV > PLOT_MAX) mV = PLOT_MAX;

  // Print: Grove_mV REF_LOW REF_HIGH
  Serial.print(mV, 2); Serial.print(" ");
  Serial.print(PLOT_MIN, 2); Serial.print(" ");
  Serial.println(PLOT_MAX, 2);

  delay(LOOP_DELAY_MS);
}
