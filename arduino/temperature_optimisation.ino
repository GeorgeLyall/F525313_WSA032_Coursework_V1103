#include <math.h>

// ── Sensor constants (Grove Temperature Sensor V1.2 datasheet)
const int    B          = 4275;   // Thermistor B-value (K)
const int    R0         = 100000; // Reference resistance (100 kΩ)
const int    SENSOR_PIN = A0;     // Analog input pin

// ── Power modes
// Named constants make the decision logic self-documenting and avoid
// magic numbers scattered throughout the control loop.
enum PowerMode { ACTIVE, IDLE, POWER_DOWN };

// ── Frequency thresholds for mode selection (from brief spec)
const float FREQ_ACTIVE_THRESHOLD = 0.5;  // Hz — above this → Active
const float FREQ_IDLE_THRESHOLD   = 0.1;  // Hz — above this → Idle, else Power Down

// ── Sampling intervals per mode (Table 1 from brief)
const unsigned long ACTIVE_INTERVAL_MS     = 1000;  //  1 s  — 1 Hz
const unsigned long IDLE_INTERVAL_MS       = 5000;  //  5 s  — 0.2 Hz
const unsigned long POWERDOWN_INTERVAL_MS  = 30000; // 30 s  — 0.033 Hz

// ── Collection parameters
// 3 minutes at 1 sample/sec (Active Mode rate from Table 1).
// Timestamps are NOT stored; time is derived as index * ACTIVE_INTERVAL_MS,
// saving 720 bytes of SRAM vs storing a parallel unsigned long array.
const int N = 180;

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


// send_data_to_pc()
//   Runs the DFT on the collected buffer then streams a combined CSV to the
//   Serial Monitor containing both time-domain and frequency-domain results.
//   Format per row: Time(ms), Temperature(C), Frequency(Hz), Magnitude
//   Each row i pairs sample i with DFT bin i so both datasets align on index.
//   The dominant frequency is printed as a summary line after the table.
void send_data_to_pc() {
  float fs       = 1000.0 / ACTIVE_INTERVAL_MS; // sampling frequency (Hz)
  float* domFreq = apply_dft();                  // populate magnitude[], get dominant f

  Serial.println("--- DATA START ---");
  Serial.println("Time(ms),Temperature(C),Frequency(Hz),Magnitude");

  for (int i = 0; i < N; i++) {
    // Time-domain column
    Serial.print((unsigned long)i * ACTIVE_INTERVAL_MS);
    Serial.print(",");
    Serial.print(tempData[i], 2);
    Serial.print(",");
    // Frequency-domain column — bin i maps to f = i * fs / N  (Eq. 3.2)
    Serial.print((float)i * fs / (float)N, 4);
    Serial.print(",");
    Serial.println(magnitude[i], 2);
  }

  Serial.println("--- DATA END ---");
  Serial.print("Dominant frequency: ");
  Serial.print(*domFreq, 4);
  Serial.println(" Hz");
}


// decide_power_mode()
//   Selects the operating mode based on the dominant frequency returned by
//   apply_dft(). Thresholds follow the brief specification:
//     > 0.5 Hz  → ACTIVE     (rapid fluctuation, high sampling needed)
//     > 0.1 Hz  → IDLE       (moderate change, reduce sampling)
//     ≤ 0.1 Hz  → POWER_DOWN (stable, minimum sampling to save energy)
//   Takes the dominant frequency in Hz and returns a PowerMode enum value.
PowerMode decide_power_mode(float dominantFreq) {
  if (dominantFreq > FREQ_ACTIVE_THRESHOLD) {
    Serial.println("Mode decision: ACTIVE (high fluctuation detected)");
    return ACTIVE;
  } else if (dominantFreq > FREQ_IDLE_THRESHOLD) {
    Serial.println("Mode decision: IDLE (moderate fluctuation detected)");
    return IDLE;
  } else {
    Serial.println("Mode decision: POWER_DOWN (temperature stable)");
    return POWER_DOWN;
  }
}


// modeIntervalMS()
//   Maps a PowerMode to its corresponding sampling interval in milliseconds.
//   Centralises the mode→interval lookup so the control loop stays readable.
unsigned long modeIntervalMS(PowerMode mode) {
  switch (mode) {
    case ACTIVE:     return ACTIVE_INTERVAL_MS;
    case IDLE:       return IDLE_INTERVAL_MS;
    case POWER_DOWN: return POWERDOWN_INTERVAL_MS;
    default:         return ACTIVE_INTERVAL_MS;
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
      send_data_to_pc();                        // DFT + full CSV output

      // Determine initial operating mode from dominant frequency
      float domFreq = *apply_dft();
      PowerMode mode = decide_power_mode(domFreq);

      Serial.print("Initial sampling interval: ");
      Serial.print(modeIntervalMS(mode));
      Serial.println(" ms");
    }
  }
  // Full adaptive control loop (moving average + dynamic rate) added in Stage 6.
}
