#include <math.h>

// ── Sensor constants (Grove Temperature Sensor V1.2 datasheet)
const int    B          = 4275;   // Thermistor B-value (K)
const int    R0         = 100000; // Reference resistance (100 kΩ)
const int    SENSOR_PIN = A0;     // Analog input pin

// ── Collection parameters
// 3 minutes at 1 sample/sec (Active Mode rate from Table 1).
// Timestamps are NOT stored; time is derived as index * ACTIVE_INTERVAL_MS,
// saving 720 bytes of SRAM vs storing a parallel unsigned long array.
const int           N                  = 180;
const unsigned long ACTIVE_INTERVAL_MS = 1000;

// ── Data buffer
float tempData[N]; // 720 bytes — the single shared buffer for all processing

// ── Collection state
static int           sampleCount    = 0;
static unsigned long lastSampleTime = 0;
static bool          collectionDone = false;


// readTemperature()
//   Converts raw ADC reading to Celsius using the Steinhart-Hart
//   B-parameter equation from the Grove sensor datasheet.
//   Returns -999 on a zero ADC reading (open-circuit guard).
float readTemperature() {
  int raw = analogRead(SENSOR_PIN);
  if (raw == 0) return -999.0;
  float R = (1023.0 / raw - 1.0) * R0;
  return 1.0 / (log(R / R0) / B + 1.0 / 298.15) - 273.15;
}


// collect_temperature_data()
//   Non-blocking sampler: must be called repeatedly from loop().
//   Fills tempData[] at ACTIVE_INTERVAL_MS without using delay(),
//   keeping the MCU free for future additions (e.g. display updates).
//   Returns true once all N samples have been collected.
bool collect_temperature_data() {
  if (sampleCount >= N) return true;

  unsigned long now = millis();
  if (now - lastSampleTime >= ACTIVE_INTERVAL_MS) {
    lastSampleTime        = now;
    tempData[sampleCount] = readTemperature();
    sampleCount++;

    // Progress update every 30 samples (every 30 s at 1 Hz)
    if (sampleCount % 30 == 0 || sampleCount == N) {
      Serial.print("Collecting: ");
      Serial.print(sampleCount);
      Serial.print(" / ");
      Serial.println(N);
    }
  }
  return (sampleCount >= N);
}


// printCollectedData()
//   Streams all collected samples over Serial in CSV format.
//   Time is reconstructed from sample index × interval — no stored timestamps.
void printCollectedData() {
  Serial.println("Time(ms),Temperature(C)");
  for (int i = 0; i < N; i++) {
    Serial.print((unsigned long)i * ACTIVE_INTERVAL_MS);
    Serial.print(",");
    Serial.println(tempData[i], 2);
  }
}


void setup() {
  Serial.begin(9600);
  Serial.println("=== Temperature Data Collection ===");
  Serial.print("Collecting ");
  Serial.print(N);
  Serial.println(" samples at 1 Hz (Active Mode)...");
}


void loop() {
  if (!collectionDone) {
    collectionDone = collect_temperature_data();

    if (collectionDone) {
      Serial.println("Collection complete.");
      printCollectedData();
    }
  }
  // Future stages (DFT, power-mode logic) will be added here.
}
