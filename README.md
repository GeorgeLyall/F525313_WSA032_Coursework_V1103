# F525313 — 25WSA032 Coursework

**Student ID:** F525313  
**Module:** 25WSA032 — Introduction to Software Engineering  
**Semester:** 2, 2025–2026  

---

## Project Overview

This repository contains all coursework deliverables for 25WSA032. The project covers four tasks spanning embedded systems programming (C/Arduino), Python optimisation, and data analytics.

| Task | Description | Deliverable | Marks |
|------|-------------|-------------|-------|
| 1 | Git source control and SDLC | `.git` repo history | 20 |
| 2 | Arduino adaptive temperature monitoring | `arduino/temperature_optimisation.ino` | 30 |
| 3 | Python robot ecosystem optimisation | `robots/robot_optimisation.py` | 30 |
| 4 | Data analytics and visualisation | Plots + Python script | 20 |

---

## Repository Structure

```
F525313_25WSA032_Coursework/
├── arduino/
│   ├── Arduino Task/          # PlatformIO project (development)
│   │   └── src/main.cpp       # Stage 1: basic sensor read
│   ├── loovee.ino             # Original demo sketch from SeeedStudio
│   └── temperature_optimisation.ino   # Task 2 deliverable
├── documentation/
│   ├── coursework brief.md
│   ├── task 1.md – task 4.md
│   ├── coursework code quality.md
│   └── Learning Outcomes.md
├── robots/
│   ├── ecosystem/
│   │   ├── ecosystem.py       # Ecosystem engine (do not modify)
│   │   ├── bots.py            # Bot base and child classes
│   │   └── factory.py        # Factory helper
│   ├── ecosystem_operation.py # Demo/baseline script
│   └── robot_optimisation.py  # Task 3 deliverable
├── data/                      # Task 4: recorded CSV data from Arduino
├── analysis/
│   └── temperature_analysis.py  # Task 4 deliverable
├── requirements.txt
└── README.md
```

---

## Setup — Install Python Dependencies

Run once from the repo root before executing any Python scripts:

```bash
pip install -r requirements.txt
```

This installs `numpy`, `matplotlib`, `pandas`, and `ipython`.

---

## Task 2 — Arduino Temperature Optimisation

### How the system works

The system implements an adaptive three-mode temperature monitoring strategy:

| Mode | Sampling rate | Trigger condition |
|------|---------------|-------------------|
| Active | 1 sample / second | Dominant frequency > 0.5 Hz or predicted variation > 0.5 °C |
| Idle | 1 sample / 5 seconds | Stable for < 5 consecutive cycles |
| Power Down | 1 sample / 30 seconds | Stable for ≥ 5 consecutive cycles |

Mode transitions use a **10-cycle moving average** of temperature variation and a **Discrete Fourier Transform** to identify dominant frequency. The sampling rate is also dynamically adjusted to be at least twice the dominant frequency (Nyquist theorem), clamped between 0.5 Hz and 4 Hz.

### Hardware wiring

| Grove Temperature Sensor V1.2 pin | Arduino connection |
|-----------------------------------|--------------------|
| VCC | 5V |
| GND | GND |
| Signal | A0 |

### Uploading the sketch

1. Open **Arduino IDE**
2. Go to **File → Open** and select `arduino/temperature_optimisation.ino`
3. Select your board: **Tools → Board → Arduino Uno**
4. Select the correct port: **Tools → Port → COMx** (check Device Manager if unsure)
5. Click **Upload** (→)
6. Open **Tools → Serial Monitor**, set baud rate to **9600**

The sketch will immediately start collecting data and print progress to the Serial Monitor. After 3 minutes (180 samples at 1 Hz) it outputs the full CSV data block between `--- DATA START ---` and `--- DATA END ---` markers, then prints the dominant frequency and selected power mode.

### Collecting data for Task 4

1. Upload the sketch and open the Serial Monitor (9600 baud)
2. Wait for the `--- DATA START ---` marker to appear (~3 minutes)
3. Select all text **between** the `--- DATA START ---` and `--- DATA END ---` lines (inclusive of the header row `Time(ms),Temperature(C),Frequency(Hz),Magnitude`)
4. Paste it into a new file and save as `data/temperature_data.csv`
5. The file should have 181 lines (1 header + 180 data rows)

**Example of expected CSV content:**

```
Time(ms),Temperature(C),Frequency(Hz),Magnitude
0,22.45,0.0000,4052.10
1000,22.47,0.0056,12.34
2000,22.46,0.0111,8.76
...
179000,22.51,0.9944,3.21
```

---

## Task 3 — Robot Ecosystem Optimisation

### How the optimisations work

`robots/robot_optimisation.py` extends the unoptimised demo with:

- **Nearest-charger routing** — 3 chargers distributed across the arena; each bot always routes to the closest one
- **Evidence-based charge thresholds** — tuned per bot type from the energy model (Robot 15%, Droid 12%, Drone 13%) vs the fixed 20% baseline
- **Opportunistic charging** — bots divert to charge if they pass within 8 arena units of a charger while at 20–55% SOC
- **Nearest-pizza allocation** — bots pick up the closest available pizza rather than the first in the list
- **Payload-aware filtering** — pizzas heavier than the bot's `max_payload` are excluded, preventing contract rejections
- **Heavier pizzas** — maximum pizza weight raised from 20 kg to 30 kg, increasing kg-delivered KPI

### Running the simulation

From the repo root:

```bash
python robots/robot_optimisation.py
```

The script runs the **baseline** configuration first, then the **optimised** configuration, and prints a side-by-side KPI comparison table showing pizzas delivered, weight delivered, distance, energy, broken bots, and damage points.

> **Note:** The default run duration is set to `2 week` for quick testing.  
> Change `DURATION = '2 week'` to `DURATION = '52 week'` in `robot_optimisation.py` for a full-year comparison before submission.

**Example output:**

```
======================================================================
KPI                      Baseline    Optimised       Change
----------------------------------------------------------------------
Pizzas delivered              142          187       +31.7%
Weight delivered (kg)        1823         2901       +59.1%
Total distance (units)      14302        11204       -21.7%
Total energy consumed        8431         6820       -19.1%
Broken bots                     2            0      -100.0%
Total damage points            14            1       -92.9%
======================================================================
```

---

## Task 4 — Data Analytics and Visualisation

### Running the analysis

From the repo root:

```bash
python analysis/temperature_analysis.py
```

The script reads `data/temperature_data.csv` and saves five plots to the `analysis/` folder:

| File | Description |
|------|-------------|
| `plot1_temperature_vs_time.png` | Raw temperature signal over 3 minutes |
| `plot2_dft_spectrum.png` | DFT magnitude spectrum with dominant frequency marked |
| `plot3_smoothed_temperature.png` | Raw signal overlaid with 10-sample moving average |
| `plot4_histogram.png` | Distribution of readings with mean and ±1σ |
| `plot5_change_rate.png` | Temperature change rate — Active Mode trigger regions highlighted |

A structured discussion of the findings (time-domain, frequency-domain, system behaviour, data quality) is printed to the console after the plots are saved.

> **No Arduino?** If `data/temperature_data.csv` does not exist, the script automatically generates a realistic synthetic dataset and saves it there, so the full workflow can be demonstrated without hardware.

### Recommended workflow (Tasks 2 → 4)

```
1.  Wire the Grove sensor to the Arduino (see wiring table above)
2.  Upload arduino/temperature_optimisation.ino
3.  Wait ~3 minutes for data collection to complete
4.  Copy CSV output from Serial Monitor → data/temperature_data.csv
5.  Run:  python analysis/temperature_analysis.py
6.  Check analysis/ folder for the five saved PNG plots
```

---

## Development Notes

- AI assistance (Claude) was used for code generation and is documented in all commit messages with `Co-Authored-By` tags.
- All core logic, parameter choices, threshold values, and design decisions are the student's own work.
- The ecosystem engine (`robots/ecosystem/`) is unmodified from the provided template.

---

## Dependencies

```bash
pip install -r requirements.txt
```

| Package | Use |
|---------|-----|
| `numpy` | DFT computation and array operations (Task 4) |
| `matplotlib` | Plotting (Task 4) |
| `pandas` | CSV reading and data manipulation (Task 4) |
| `ipython` | Interactive development support |
