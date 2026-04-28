#include <math.h>

// ── Sensor constants (from Grove datasheet)
const int   B             = 4275;   // Thermistor B-value (K)
const long  R0            = 100000; // Reference resistance (100 kΩ)
const int   SENSOR_PIN    = A0;     // Analog input pin

// ── Sampling config 
const unsigned long SAMPLE_INTERVAL_MS = 1000; // 1 sample/sec (Active Mode rate)

// ── Timing state
unsigned long lastSampleTime = 0;



// readTemperature()
//   Reads the raw ADC value from the Grove thermistor and converts
//   it to degrees Celsius using the Steinhart-Hart B-parameter eq.
//   Returns: temperature in °C as a float

float readTemperature() {
  int   raw = analogRead(SENSOR_PIN);

  // Convert ADC reading → resistance → temperature
  float R   = (1023.0 / raw - 1.0) * R0;
  float T   = 1.0 / (log(R / R0) / B + 1.0 / 298.15) - 273.15;

  return T;
}



// setup()
//   Initialises Serial communication at 9600 baud and prints a
//   CSV header so output can be pasted straight into a spreadsheet.

void setup() {
  Serial.begin(9600);
  Serial.println(F("Time(ms),Temperature(C)"));
}



// loop()
//   Non-blocking sample loop — uses millis() rather than delay()
//   so the Arduino remains responsive for future additions.

void loop() {
  unsigned long now = millis();

  if (now - lastSampleTime >= SAMPLE_INTERVAL_MS) {
    lastSampleTime = now;

    float temp = readTemperature();

    Serial.print(now);
    Serial.print(F(","));
    Serial.println(temp);
  }
}