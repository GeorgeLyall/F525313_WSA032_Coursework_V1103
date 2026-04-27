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
├── analysis/                  # Task 4: Python plotting scripts
├── requirements.txt
└── README.md
```

---

## Task 2 — Arduino Temperature Optimisation

The system implements an adaptive three-mode temperature monitoring strategy:

- **Active Mode** — 1 sample/second; used when temperature is changing rapidly
- **Idle Mode** — 1 sample every 5 seconds; used during minor fluctuations
- **Power Down Mode** — 1 sample every 30 seconds; used when temperature is stable

Mode transitions are driven by:
1. A **moving average** of temperature differences over the last 10 readings
2. A **Discrete Fourier Transform (DFT)** to identify dominant change frequencies
3. **Nyquist-compliant** dynamic sampling: rate ≥ 2× dominant frequency

### How to run (Arduino IDE)
1. Open `arduino/temperature_optimisation.ino` in Arduino IDE
2. Connect Grove Temperature Sensor V1.2 to pin A0 (VCC → 5V, GND → GND, Signal → A0)
3. Upload to Arduino Uno
4. Open Serial Monitor at 9600 baud to view CSV output

---

## Task 3 — Robot Ecosystem Optimisation

`robots/robot_optimisation.py` extends the demo to optimise bot performance via:

- **Nearest-charger selection** — bots route to the closest of multiple chargers
- **Evidence-based charge thresholds** — tuned per bot type (Robot / Droid / Drone)
- **Opportunistic charging** — bots divert to charge when passing near a charger
- **Proximity-based pizza allocation** — bots collect the nearest available pizza
- **Payload-aware allocation** — prevents overweight contracts being rejected

Run from the repo root:

```bash
python robots/robot_optimisation.py
```

---

## Task 4 — Data Analytics

`analysis/temperature_analysis.py` reads temperature data recorded from the Arduino system and produces five plots:

1. Temperature vs Time
2. DFT Magnitude vs Frequency
3. Original vs smoothed (moving average) temperature
4. Histogram of temperature readings
5. Temperature change rate vs Time

Data is stored in `data/temperature_data.csv`.

---

## Development Notes

- AI assistance was used for code generation and is documented in commit messages.
- All core logic, parameter choices, and design decisions are the student's own.
- The ecosystem engine (`robots/ecosystem/`) is unmodified from the provided template.

---

## Dependencies

```
pip install -r requirements.txt
```

See `requirements.txt` for the full list (matplotlib, numpy, pandas).
