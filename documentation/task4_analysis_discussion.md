# Task 4 — Analysis Discussion

**Student:** George Lyall &nbsp; | &nbsp; **ID:** F525313 &nbsp; | &nbsp; **Module:** 25WSA032

---

## Dataset

Temperature data was recorded using the Arduino temperature monitoring system (`arduino/temperature_optimisation.ino`) over a 3-minute session at 1 Hz, yielding 180 samples. Data was saved to `data/temperature_data.csv` and analysed using `analysis/temperature_analysis.py`.

---

## Time-Domain Behaviour (Plot 1, Plot 3, Plot 5)

**Plot 1 — Temperature vs Time** shows the raw signal over the 3-minute recording. The temperature remained stable throughout, staying within a narrow range of approximately ±0.2 °C of the mean. No sudden large rises or drops were observed.

**Plot 3 — Raw vs Smoothed Temperature** overlays the 10-sample centred moving average on the raw signal. The smoothed trace closely follows the raw signal, confirming that no significant low-frequency trend was hidden by noise and that the raw signal was already close to the underlying process. The small residual between raw and smoothed is consistent with sensor quantisation noise (~0.1 °C from the 10-bit ADC).

**Plot 5 — Temperature Change Rate** shows the rate of change between consecutive samples. The vast majority of samples have a rate well below 0.02 °C/s. Isolated spikes above this threshold are caused by quantisation step changes in the ADC output and do not represent genuine temperature transitions. In the adaptive system, these would momentarily trigger Active Mode before the 10-cycle moving average smoothed them out.

---

## Frequency-Domain Behaviour (Plot 2)

**Plot 2 — DFT Magnitude Spectrum** shows the positive-frequency bins from the DFT of the mean-centred signal. The dominant non-DC frequency has a very low value (< 0.1 Hz), indicating the signal is almost entirely low-frequency.

This result is consistent with an indoor environment where temperature changes occur over minutes or tens of minutes, not sub-second cycles. Higher-frequency bins show near-zero magnitude, confirming that sensor noise does not introduce significant periodic components that would corrupt the mode-selection logic.

The DC component (k = 0) was excluded from mode selection in the Arduino code because it represents the mean temperature level rather than a fluctuation — including it would always select Active Mode regardless of signal content.

---

## System Behaviour

The dominant frequency being below 0.1 Hz means the Arduino system would correctly select **Power Down Mode** (1 sample every 30 seconds) for the majority of normal indoor operation. This is a 97% reduction in sample volume compared to Active Mode (86 400 samples/day vs 2 880/day), which is the primary energy-saving benefit of the adaptive strategy.

The 10-cycle stability counter before entering Power Down Mode provides a suitable hysteresis: brief disturbances (e.g. a door opening) will temporarily raise the mode to Idle or Active but will not prevent the system from returning to Power Down once conditions stabilise.

**Potential improvement:** the Idle → Power Down countdown threshold of 5 consecutive stable cycles could be shortened to 3 when the moving-average standard deviation over those cycles is very small (< 0.05 °C), allowing faster energy savings without loss of detection sensitivity.

---

## Data Quality

The 3-minute recording at 1 Hz resolves frequencies from approximately 0.006 Hz (one cycle per recording) to 0.5 Hz (Nyquist limit). This range is appropriate for indoor HVAC and human-activity temperature cycles, which typically have periods of tens of seconds to minutes.

**Limitations:**
- The 10-bit ADC introduces a quantisation step of roughly 0.1 °C, which raises the noise floor in all DFT bins and produces the isolated spikes visible in Plot 5.
- Three minutes captures only about 0.5 cycles of a 5-minute oscillation — longer recordings would improve frequency resolution for slow processes.
- Oversampling (e.g. 4× at 4 Hz, then averaging to 1 Hz) would reduce quantisation noise by approximately a factor of two without any hardware changes.

---

*Plots saved to `analysis/`: `plot1_temperature_vs_time.png`, `plot2_dft_spectrum.png`, `plot3_smoothed_temperature.png`, `plot4_histogram.png`, `plot5_change_rate.png`.*
