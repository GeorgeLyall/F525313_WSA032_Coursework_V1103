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

// ── Data buffers
// tempData  : raw temperature samples          (180 × 4 = 720 bytes)
// magnitude : DFT magnitude per frequency bin  (180 × 4 = 720 bytes)
// Total: 1 440 bytes of the 2 048-byte Uno SRAM — leaves ~600 bytes for stack.
// real[] and imag[] are NOT stored globally; each bin is computed as running
// scalar sums, so only 8 bytes of extra stack are needed during the DFT.
float tempData[N];
float magnitude[N];

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


// apply_dft()
//   Implements the DFT (Eqs 3.3–3.5) on the contents of tempData[].
//   For each frequency bin k, real and imaginary parts are accumulated as
//   scalar running sums (not stored arrays) to minimise SRAM use.
//   Magnitude[k] is stored in the global magnitude[] array (Eq. 3.5).
//   The DC bin (k = 0) is excluded from dominant-frequency detection because
//   it represents the mean temperature level, not a fluctuation.
//   Returns a pointer to a static float holding the dominant frequency in Hz.
float* apply_dft() {
  static float dominantFreq = 0.0;

  // Sampling frequency derived from the collection interval
  float fs     = 1000.0 / ACTIVE_INTERVAL_MS; // Hz  (1.0 Hz in Active Mode)
  float maxMag = 0.0;
  int   domK   = 1; // start at 1 — skip DC

  for (int k = 0; k < N; k++) {
    float re = 0.0, im = 0.0;

    // Accumulate real and imaginary parts (Eqs 3.3 & 3.4)
    for (int n = 0; n < N; n++) {
      float angle = 2.0 * M_PI * (float)k * (float)n / (float)N;
      re +=  tempData[n] * cos(angle);
      im -=  tempData[n] * sin(angle);
    }

    // Magnitude for this bin (Eq. 3.5)
    magnitude[k] = sqrt(re * re + im * im);

    // Track peak bin, ignoring DC (k = 0)
    if (k > 0 && magnitude[k] > maxMag) {
      maxMag = magnitude[k];
      domK   = k;
    }
  }

  // Convert bin index to Hz (Eq. 3.2): f_k = k * fs / N
  dominantFreq = (float)domK * fs / (float)N;
  return &dominantFreq;
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

      // Run DFT and report dominant frequency
      float* domFreq = apply_dft();
      Serial.print("Dominant frequency: ");
      Serial.print(*domFreq, 4);
      Serial.println(" Hz");
    }
  }
  // send_data_to_pc() and decide_power_mode() added in next stages.
}
