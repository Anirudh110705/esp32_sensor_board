// ===================== MAX9814 Sound Sensor Plot =====================
// ESP32 + MAX9814 (analog OUT)
// Plots Vrms (mV) and Sound Level (dB) on Serial Plotter
// Includes fixed REF lines to prevent autoscaling

const int MAX9814_PIN = 35;   // Analog pin connected to MAX9814 OUT

// ADC / Timing
const float VREF = 3.3f;
const int ADC_RES = 4095;
const int SAMPLE_RATE = 2000;       // Hz for RMS calculation
const int SAMPLE_WINDOW_MS = 50;
const int LOOP_DELAY_MS = 200;

// dB calibration
const float refVrms = 0.02f;  // reference Vrms
const float refDb   = 40.0f;  // reference dB
const float GAIN_FACTOR = 1.3f;
const float CALIB_OFFSET = 10.0f;

// Plot bounds
const float PLOT_DB_MIN = 20.0f;
const float PLOT_DB_MAX = 100.0f;

// ---------------- RMS Calculation ----------------
float measureVrms(int pin, int windowMs) {
  unsigned long start = millis();
  unsigned long sampleIntervalUs = 1000000UL / SAMPLE_RATE;
  unsigned long nextMicros = micros();
  unsigned long count = 0;
  double sum = 0.0, sumsq = 0.0;

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
  double mean = sum / count;
  double meanSq = sumsq / count;
  double vrms = sqrt(fmax(0.0, meanSq - mean * mean));
  return (float)vrms;
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);

  Serial.println("DB Vrms_mV REF_LOW REF_HIGH");  // For Serial Plotter
}

void loop() {
  float vrms = measureVrms(MAX9814_PIN, SAMPLE_WINDOW_MS);
  float dB = -999.0f;
  if (vrms > 0.0f) {
    dB = refDb + GAIN_FACTOR * (20.0f * log10f(vrms / refVrms)) - CALIB_OFFSET;
  }

  // Clamp values to avoid autoscaling
  if (dB < PLOT_DB_MIN) dB = PLOT_DB_MIN;
  if (dB > PLOT_DB_MAX) dB = PLOT_DB_MAX;

  float vrms_mV = vrms * 1000.0f;

  // Print line: dB Vrms REF_LOW REF_HIGH
  Serial.print(dB, 2); Serial.print(" ");
  Serial.print(vrms_mV, 2); Serial.print(" ");
  Serial.print(PLOT_DB_MIN, 2); Serial.print(" ");
  Serial.println(PLOT_DB_MAX, 2);

  delay(LOOP_DELAY_MS);
}
