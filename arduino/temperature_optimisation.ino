#include <math.h>

// ─────────────────────────────────────────────────────────────────────────────
// Sensor constants — Grove Temperature Sensor V1.2 (Steinhart-Hart B-parameter)
// ─────────────────────────────────────────────────────────────────────────────
const int B          = 4275;   // Thermistor B-value (K)
const int R0         = 100000; // Reference resistance (100 kΩ)
const int SENSOR_PIN = A0;     // Analog input pin

// ─────────────────────────────────────────────────────────────────────────────
// Power mode definitions
// ─────────────────────────────────────────────────────────────────────────────
enum PowerMode { ACTIVE, IDLE, POWER_DOWN };

// Frequency thresholds for mode selection (brief specification)
const float FREQ_ACTIVE_THRESHOLD = 0.5;  // Hz — above this → Active
const float FREQ_IDLE_THRESHOLD   = 0.1;  // Hz — above this → Idle, else Power Down

// Sampling intervals per mode (Table 1 from brief)
const unsigned long ACTIVE_INTERVAL_MS    = 1000;  //  1 s  (1.0 Hz)
const unsigned long IDLE_INTERVAL_MS      = 5000;  //  5 s  (0.2 Hz)
const unsigned long POWERDOWN_INTERVAL_MS = 30000; // 30 s  (0.033 Hz)

// Dynamic rate bounds applied in ACTIVE mode to satisfy Nyquist (brief spec)
const unsigned long MIN_INTERVAL_MS = 250;  // 4.0 Hz upper bound
const unsigned long MAX_INTERVAL_MS = 2000; // 0.5 Hz lower bound

// ─────────────────────────────────────────────────────────────────────────────
// Adaptive-loop parameters
// ─────────────────────────────────────────────────────────────────────────────
// Each cycle collects data for one minute then re-evaluates the power mode.
const unsigned long CYCLE_DURATION_MS = 60000;

// Variation threshold (°C summed across all consecutive differences per cycle).
// 0.5°C chosen as clearly above sensor noise (~0.1°C) but below meaningful
// room temperature change — tune upward if false Active triggers occur.
const float VARIATION_THRESHOLD = 0.5;

// Consecutive Idle cycles required before dropping to Power Down (brief: 5)
const int IDLE_CYCLES_FOR_POWERDOWN = 5;

// Moving-average window for trend prediction (brief specifies 5–10 cycles)
const int MA_WINDOW = 10;

// ─────────────────────────────────────────────────────────────────────────────
// Memory layout — Arduino Uno SRAM budget (2 048 bytes total)
//
//   tempData[180]         180 × 4 =  720 bytes   raw temperature samples
//   magnitude[180]        180 × 4 =  720 bytes   DFT magnitude per bin
//   variationHistory[10]   10 × 4 =   40 bytes   moving-average ring buffer
//   scalar globals                 ~  50 bytes
//   ───────────────────────────────────────────
//   Total static                  ~1530 bytes   leaves ~518 bytes for stack
//
// Key optimisation applied:
//   real[k] and imag[k] are computed as scalar running sums (not arrays),
//   saving 2 × 720 = 1 440 bytes compared with a naive DFT implementation.
//
// Further optimisation possible (not applied — required by brief):
//   Removing magnitude[] and tracking only peak magnitude inline during the
//   DFT pass would save another 720 bytes. This is viable in a production
//   build where send_data_to_pc() serial output is disabled.
//   Storing large float arrays on the Uno is the primary SRAM risk: if
//   static + stack together exceed 2 048 bytes the sketch silently corrupts
//   memory, causing unpredictable behaviour at runtime.
// ─────────────────────────────────────────────────────────────────────────────
const int N = 180; // buffer size; each cycle uses min(CYCLE_DURATION_MS / interval, N)

float tempData[N];
float magnitude[N];
float variationHistory[MA_WINDOW];

// ─────────────────────────────────────────────────────────────────────────────
// Control state (persists across cycles)
// ─────────────────────────────────────────────────────────────────────────────
unsigned long sampleInterval = ACTIVE_INTERVAL_MS; // current sampling interval
PowerMode     currentMode    = ACTIVE;
int           idleCycleCount = 0;
int           maIndex        = 0;    // ring-buffer write head
bool          maFull         = false; // true once all MA_WINDOW slots used

// Per-cycle counters (reset at the start of every cycle)
int           sampleCount    = 0;
int           cycleSamples   = 0;    // target samples for the current cycle
unsigned long lastSampleTime = 0;

// ─────────────────────────────────────────────────────────────────────────────
// readTemperature()
//   Converts raw ADC reading to Celsius using the Steinhart-Hart
//   B-parameter equation (Grove sensor datasheet).
//   Returns -999 on a zero ADC reading (open-circuit guard).
// ─────────────────────────────────────────────────────────────────────────────
float readTemperature() {
  int raw = analogRead(SENSOR_PIN);
  if (raw == 0) return -999.0;
  float R = (1023.0 / raw - 1.0) * R0;
  return 1.0 / (log(R / R0) / B + 1.0 / 298.15) - 273.15;
}

// ─────────────────────────────────────────────────────────────────────────────
// collect_temperature_data()
//   Non-blocking sampler — call repeatedly from loop().
//   Fills tempData[0..cycleSamples-1] at the current sampleInterval.
//   Returns true when the cycle's quota of samples has been collected.
// ─────────────────────────────────────────────────────────────────────────────
bool collect_temperature_data() {
  if (sampleCount >= cycleSamples) return true;

  unsigned long now = millis();
  if (now - lastSampleTime >= sampleInterval) {
    lastSampleTime        = now;
    tempData[sampleCount] = readTemperature();
    sampleCount++;

    if (sampleCount % 10 == 0 || sampleCount == cycleSamples) {
      Serial.print("Collecting: ");
      Serial.print(sampleCount);
      Serial.print(" / ");
      Serial.println(cycleSamples);
    }
  }
  return (sampleCount >= cycleSamples);
}

// ─────────────────────────────────────────────────────────────────────────────
// apply_dft()
//   Implements DFT (Eqs 3.3–3.5) on tempData[0..count-1].
//   real and imag are scalar accumulators (not arrays) — saves 1 440 bytes.
//   magnitude[k] is stored for all bins (used by send_data_to_pc).
//   DC bin (k=0) excluded from dominant-frequency detection per brief spec.
//   Returns pointer to a static float holding the dominant frequency in Hz.
// ─────────────────────────────────────────────────────────────────────────────
float* apply_dft(int count) {
  static float dominantFreq = 0.0;
  if (count < 2) { dominantFreq = 0.0; return &dominantFreq; }

  float fs     = 1000.0 / sampleInterval; // current sampling frequency (Hz)
  float maxMag = 0.0;
  int   domK   = 1;

  for (int k = 0; k < count; k++) {
    float re = 0.0, im = 0.0;
    for (int n = 0; n < count; n++) {
      float angle = 2.0 * M_PI * (float)k * (float)n / (float)count;
      re +=  tempData[n] * cos(angle); // Eq. 3.3
      im -=  tempData[n] * sin(angle); // Eq. 3.4
    }
    magnitude[k] = sqrt(re * re + im * im); // Eq. 3.5

    if (k > 0 && magnitude[k] > maxMag) {
      maxMag = magnitude[k];
      domK   = k;
    }
  }

  dominantFreq = (float)domK * fs / (float)count; // Eq. 3.2
  return &dominantFreq;
}

// ─────────────────────────────────────────────────────────────────────────────
// send_data_to_pc()
//   Calls apply_dft() then streams a combined CSV to the Serial Monitor.
//   Format: Time(ms), Temperature(C), Frequency(Hz), Magnitude
//   Only outputs rows for the samples collected this cycle (count entries).
// ─────────────────────────────────────────────────────────────────────────────
void send_data_to_pc(int count) {
  float fs       = 1000.0 / sampleInterval;
  float* domFreq = apply_dft(count);

  Serial.println("--- DATA START ---");
  Serial.println("Time(ms),Temperature(C),Frequency(Hz),Magnitude");
  for (int i = 0; i < count; i++) {
    Serial.print((unsigned long)i * sampleInterval);
    Serial.print(",");
    Serial.print(tempData[i], 2);
    Serial.print(",");
    Serial.print((float)i * fs / (float)count, 4); // Eq. 3.2 per bin
    Serial.print(",");
    Serial.println(magnitude[i], 2);
  }
  Serial.println("--- DATA END ---");
  Serial.print("Dominant frequency: ");
  Serial.print(*domFreq, 4);
  Serial.println(" Hz");
}

// ─────────────────────────────────────────────────────────────────────────────
// decide_power_mode()
//   Selects operating mode from dominant frequency (brief thresholds).
//   > 0.5 Hz → ACTIVE | > 0.1 Hz → IDLE | ≤ 0.1 Hz → POWER_DOWN
// ─────────────────────────────────────────────────────────────────────────────
PowerMode decide_power_mode(float dominantFreq) {
  if (dominantFreq > FREQ_ACTIVE_THRESHOLD) {
    return ACTIVE;
  } else if (dominantFreq > FREQ_IDLE_THRESHOLD) {
    return IDLE;
  } else {
    return POWER_DOWN;
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// computeVariation()
//   Returns the sum of absolute differences between consecutive samples.
//   This single number characterises how much temperature changed in one cycle.
// ─────────────────────────────────────────────────────────────────────────────
float computeVariation(int count) {
  float total = 0.0;
  for (int i = 1; i < count; i++) {
    total += fabs(tempData[i] - tempData[i - 1]);
  }
  return total;
}

// ─────────────────────────────────────────────────────────────────────────────
// updateMovingAverage() / predictedVariation()
//   Ring buffer of the last MA_WINDOW cycle variations.
//   updateMovingAverage() inserts a new value.
//   predictedVariation() returns the mean, predicting the next cycle's change.
// ─────────────────────────────────────────────────────────────────────────────
void updateMovingAverage(float variation) {
  variationHistory[maIndex] = variation;
  maIndex = (maIndex + 1) % MA_WINDOW;
  if (maIndex == 0) maFull = true;
}

float predictedVariation() {
  int   entries = maFull ? MA_WINDOW : maIndex;
  if (entries == 0) return 0.0;
  float sum = 0.0;
  for (int i = 0; i < entries; i++) sum += variationHistory[i];
  return sum / entries;
}

// ─────────────────────────────────────────────────────────────────────────────
// adjustSamplingRate()
//   Sets sampleInterval for the next cycle.
//   In ACTIVE mode: applies Nyquist (fs ≥ 2 × domFreq) then clamps to the
//   0.5–4 Hz dynamic range from the brief. Gradual decrease when stable.
//   In IDLE / POWER_DOWN: uses the fixed Table 1 intervals.
// ─────────────────────────────────────────────────────────────────────────────
void adjustSamplingRate(PowerMode mode, float domFreq) {
  if (mode == ACTIVE) {
    unsigned long nyquistInterval = (domFreq > 0.0)
      ? (unsigned long)(1000.0 / (2.0 * domFreq))  // ms for Nyquist rate
      : MAX_INTERVAL_MS;

    // Clamp to [MIN_INTERVAL_MS, MAX_INTERVAL_MS] (4 Hz – 0.5 Hz)
    if (nyquistInterval < MIN_INTERVAL_MS) nyquistInterval = MIN_INTERVAL_MS;
    if (nyquistInterval > MAX_INTERVAL_MS) nyquistInterval = MAX_INTERVAL_MS;
    sampleInterval = nyquistInterval;

  } else if (mode == IDLE) {
    sampleInterval = IDLE_INTERVAL_MS;
  } else {
    sampleInterval = POWERDOWN_INTERVAL_MS;
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// startCycle()
//   Resets per-cycle state and computes how many samples to collect.
//   Caps at N to prevent buffer overflow if rate is very high.
// ─────────────────────────────────────────────────────────────────────────────
void startCycle() {
  sampleCount  = 0;
  int target   = (int)(CYCLE_DURATION_MS / sampleInterval);
  cycleSamples = (target > N) ? N : target; // never exceed buffer size
  Serial.print("\n=== New cycle | Mode: ");
  Serial.print(currentMode == ACTIVE ? "ACTIVE" : currentMode == IDLE ? "IDLE" : "POWER_DOWN");
  Serial.print(" | Interval: ");
  Serial.print(sampleInterval);
  Serial.print(" ms | Samples: ");
  Serial.println(cycleSamples);
}

// ─────────────────────────────────────────────────────────────────────────────
// analyseCycle()
//   Called once per completed cycle. Runs DFT, updates moving average,
//   decides the next power mode, and adjusts the sampling rate.
// ─────────────────────────────────────────────────────────────────────────────
void analyseCycle() {
  // Send all data to PC first (DFT runs inside send_data_to_pc)
  send_data_to_pc(sampleCount);
  float domFreq = *apply_dft(sampleCount); // re-use already-computed result

  // Variation-based mode decision
  float variation = computeVariation(sampleCount);
  updateMovingAverage(variation);
  float predicted = predictedVariation();

  Serial.print("Cycle variation: ");   Serial.print(variation, 3);   Serial.println(" °C");
  Serial.print("Predicted trend: ");   Serial.print(predicted, 3);   Serial.println(" °C");

  if (predicted > VARIATION_THRESHOLD) {
    // Temperature is actively changing — force Active regardless of frequency
    currentMode    = ACTIVE;
    idleCycleCount = 0;
  } else {
    // Stable — consult DFT for finer mode selection
    PowerMode freqMode = decide_power_mode(domFreq);

    if (freqMode == ACTIVE) {
      currentMode    = ACTIVE;
      idleCycleCount = 0;
    } else {
      // Both variation and frequency indicate stability
      idleCycleCount++;
      currentMode = (idleCycleCount >= IDLE_CYCLES_FOR_POWERDOWN) ? POWER_DOWN : IDLE;
    }
  }

  adjustSamplingRate(currentMode, domFreq);

  Serial.print("Next mode: ");
  Serial.print(currentMode == ACTIVE ? "ACTIVE" : currentMode == IDLE ? "IDLE" : "POWER_DOWN");
  Serial.print(" | Next interval: ");
  Serial.print(sampleInterval);
  Serial.println(" ms");
}


// ─────────────────────────────────────────────────────────────────────────────
// setup() / loop()
// ─────────────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(9600);
  Serial.println("=== Adaptive Temperature Monitoring System ===");
  startCycle();
}

void loop() {
  if (collect_temperature_data()) {
    analyseCycle(); // DFT + mode decision + rate adjustment
    startCycle();   // reset counters and begin next cycle
  }
}
