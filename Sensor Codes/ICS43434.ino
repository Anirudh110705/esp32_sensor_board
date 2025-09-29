// ICS43434 -> ESP32 : Vrms (normalized 0..1) and dBFS
// Prints: vrms_norm dBFS threshold
// Serial Plotter will plot vrms_norm and threshold line.

#include <Arduino.h>
#include "driver/i2s.h"
#include <math.h>

#define I2S_NUM             I2S_NUM_0
#define I2S_SAMPLE_RATE     16000
#define I2S_BITS            I2S_BITS_PER_SAMPLE_32BIT

// I2S pins (change if needed)
#define I2S_BCK_IO   26
#define I2S_WS_IO    25
#define I2S_DO_IO    27

#define DATA_LEFT_ALIGNED  true   // set false for right-aligned breakouts

const int BUFFER_SAMPLES = 256;
static int32_t i2s_buffer[BUFFER_SAMPLES];
const int SAMPLES_PER_MEAS = 256;
const double FS_24 = 8388607.0;   // max 24-bit signed value

// Threshold (in normalized Vrms units, 0..1)
const float VRMS_THRESHOLD = 0.3f;   // adjust depending on noise floor
const double FIXED_DB_THRESHOLD = 100.0; 

// I2S setup
void setupI2S() {
  i2s_config_t cfg = {};
  cfg.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX);
  cfg.sample_rate = I2S_SAMPLE_RATE;
  cfg.bits_per_sample = (i2s_bits_per_sample_t)I2S_BITS;
  cfg.channel_format = I2S_CHANNEL_FMT_ONLY_LEFT;
  cfg.communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB);
  cfg.intr_alloc_flags = 0;
  cfg.dma_buf_count = 4;
  cfg.dma_buf_len = BUFFER_SAMPLES;
  cfg.use_apll = false;
  cfg.tx_desc_auto_clear = false;
  cfg.fixed_mclk = 0;

  i2s_pin_config_t pins = {};
  pins.bck_io_num = I2S_BCK_IO;
  pins.ws_io_num = I2S_WS_IO;
  pins.data_out_num = I2S_PIN_NO_CHANGE;
  pins.data_in_num = I2S_DO_IO;

  esp_err_t err = i2s_driver_install(I2S_NUM, &cfg, 0, NULL);
  if (err != ESP_OK) {
    Serial.printf("i2s_driver_install failed: %d\n", err);
    while (1) delay(1000);
  }
  i2s_set_pin(I2S_NUM, &pins);
  i2s_set_sample_rates(I2S_NUM, I2S_SAMPLE_RATE);
}

int32_t extract24(int32_t raw) {
  if (DATA_LEFT_ALIGNED) {
    return raw >> 8;   // shift down to 24-bit signed
  } else {
    int32_t v = raw & 0x00FFFFFF;
    if (v & 0x00800000) v |= 0xFF000000;
    return v;
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(5);
  setupI2S();
  Serial.println("vrms_norm dBFS threshold"); // header for Serial Plotter
}

void loop() {
  size_t bytes_read = 0;
  esp_err_t r = i2s_read(I2S_NUM, (void*)i2s_buffer, BUFFER_SAMPLES * sizeof(int32_t), &bytes_read, pdMS_TO_TICKS(500));
  if (r != ESP_OK || bytes_read == 0) {
    delay(10);
    return;
  }
  int samples = bytes_read / sizeof(int32_t);
  if (samples < SAMPLES_PER_MEAS) {
    delay(5);
    return;
  }

  double sumSq = 0.0;
  for (int i = 0; i < SAMPLES_PER_MEAS; ++i) {
    int32_t raw = i2s_buffer[i];
    int32_t s24 = extract24(raw);
    double norm = (double)s24 / FS_24;
    sumSq += norm * norm;
  }
  double vrms_norm = sqrt(sumSq / SAMPLES_PER_MEAS); // normalized Vrms (0..1)
  double dBFS = (vrms_norm > 0.0) ? 20.0 * log10(vrms_norm) : -120;
  double dBSPL = dBFS + 94+26; 
  // Print for Serial Plotter: vrms_norm, dBFS, and threshold line
  Serial.print(vrms_norm, 6);
  Serial.print(" ");
  Serial.print(dBSPL, 2);
  Serial.print(" ");
  Serial.print(VRMS_THRESHOLD, 6);
  Serial.print(" ");
  Serial.println(FIXED_DB_THRESHOLD, 2);
 

  delay(60); // ~16 updates/sec
}
