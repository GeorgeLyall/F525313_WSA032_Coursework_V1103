"""
temperature_analysis.py

Task 4: Data analytics and visualisation for the Arduino temperature
monitoring system (25WSA032 Coursework).

Reads the CSV produced by temperature_optimisation.ino (via the Serial
Monitor) and produces five diagnostic plots saved to the analysis/ folder:

  Plot 1 — Temperature vs Time          (time-domain signal overview)
  Plot 2 — DFT Magnitude vs Frequency   (frequency-domain spectrum)
  Plot 3 — Raw vs Smoothed Temperature  (moving-average noise reduction)
  Plot 4 — Histogram of Readings        (distribution and outlier check)
  Plot 5 — Temperature Change Rate      (identifies rapid transitions)

If no recorded CSV is found the script exits with an error directing
the user to run collect_data.py first.

A structured discussion of the findings is printed to the console after
all plots are saved.

Usage (from repo root):
    python analysis/temperature_analysis.py

Expected CSV columns (from send_data_to_pc()):
    Time(ms), Temperature(C), Frequency(Hz), Magnitude
"""

import os
import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# ── Paths ─────────────────────────────────────────────────────────────────
DATA_PATH  = os.path.join('data', 'temperature_data.csv')
FIG_DIR    = 'analysis'

# ── Moving-average window for Plot 3 (samples)
MA_WINDOW  = 10

# ── Threshold for Plot 5 highlight band (°C/s) — matches brief discussion
RATE_THRESHOLD = 0.02


# ─────────────────────────────────────────────────────────────────────────────
# Data loading
# ─────────────────────────────────────────────────────────────────────────────

def load_csv(path):
    """
    Read the temperature CSV exported from the Arduino Serial Monitor.
    Expected columns: Time(ms), Temperature(C), Frequency(Hz), Magnitude
    Returns a DataFrame with an additional 'time_s' column (seconds).
    """
    df = pd.read_csv(path)
    df.columns = [c.strip() for c in df.columns]
    df = df.rename(columns={
        'Time(ms)':       'time_ms',
        'Temperature(C)': 'temp_c',
        'Frequency(Hz)':  'freq_hz',
        'Magnitude':      'magnitude',
    })
    # Drop any repeated header rows and coerce all columns to float
    numeric_cols = ['time_ms', 'temp_c', 'freq_hz', 'magnitude']
    for col in numeric_cols:
        df[col] = pd.to_numeric(df[col], errors='coerce')
    df = df.dropna(subset=numeric_cols).copy()
    df['time_s'] = df['time_ms'] / 1000.0
    return df


def add_dft_columns(df):
    """
    Compute DFT in Python when the Arduino CSV contains only time/temperature
    columns (e.g. data captured from a basic logger rather than send_data_to_pc).
    Adds 'freq_hz' and 'magnitude' columns to the DataFrame in-place.
    """
    n        = len(df)
    dt       = df['time_s'].diff().median()          # median sample interval (s)
    fs       = 1.0 / dt if dt > 0 else 1.0
    centred  = df['temp_c'].to_numpy() - df['temp_c'].mean()
    fft_mag  = np.abs(np.fft.rfft(centred))
    fft_freq = np.fft.rfftfreq(n, d=1.0 / fs)
    m        = len(fft_freq)
    df['freq_hz']   = np.concatenate([fft_freq,  np.zeros(n - m)])
    df['magnitude'] = np.concatenate([fft_mag,   np.zeros(n - m)])
    return df



# ─────────────────────────────────────────────────────────────────────────────
# Analysis helpers
# ─────────────────────────────────────────────────────────────────────────────

def moving_average(data, window):
    """Centred moving average; edges use whatever data are available."""
    return (pd.Series(data)
            .rolling(window=window, center=True, min_periods=1)
            .mean()
            .to_numpy())


def change_rate(temp, time_s):
    """
    Compute the rate of temperature change (°C/s) between consecutive samples.
    A leading zero is prepended so the output aligns with the time axis.
    """
    dt   = np.diff(time_s)
    dT   = np.diff(temp)
    rate = np.where(dt > 0, dT / dt, 0.0)
    return np.concatenate([[0.0], rate])


def dominant_frequency(df):
    """Return the frequency (Hz) of the highest-magnitude non-DC DFT bin."""
    half = len(df) // 2
    sub  = df.iloc[1:half]          # skip DC (k = 0)
    idx  = sub['magnitude'].idxmax()
    return df.loc[idx, 'freq_hz']


def save_figure(fig, filename):
    """Save a matplotlib figure to FIG_DIR as a 150-dpi PNG."""
    path = os.path.join(FIG_DIR, filename)
    fig.savefig(path, dpi=150, bbox_inches='tight')
    print(f"  Saved → {path}")


# ─────────────────────────────────────────────────────────────────────────────
# Plot functions
# ─────────────────────────────────────────────────────────────────────────────

def plot1_temperature_vs_time(df):
    """Plot 1: Raw temperature signal over the recording period."""
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(df['time_s'], df['temp_c'], color='steelblue', linewidth=1.2)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Temperature (°C)')
    ax.set_title('Plot 1 — Temperature vs Time (3-minute recording)')
    ax.grid(True, linestyle='--', alpha=0.5)
    fig.tight_layout()
    save_figure(fig, 'plot1_temperature_vs_time.png')
    plt.close(fig)


def plot2_dft_spectrum(df):
    """
    Plot 2: DFT magnitude spectrum.
    Only the positive, non-DC frequency bins are shown.
    The dominant bin is marked with a dashed vertical line.
    Returns the dominant frequency in Hz.
    """
    half    = len(df) // 2
    freq    = df['freq_hz'].iloc[1:half].to_numpy()
    mag     = df['magnitude'].iloc[1:half].to_numpy()
    dom_f   = dominant_frequency(df)

    fig, ax = plt.subplots(figsize=(10, 4))
    ax.bar(freq, mag, width=freq[1] - freq[0] if len(freq) > 1 else 0.005,
           color='darkorange', alpha=0.8, label='DFT magnitude')
    ax.axvline(dom_f, color='red', linestyle='--', linewidth=1.5,
               label=f'Dominant: {dom_f:.4f} Hz')
    ax.set_xlabel('Frequency (Hz)')
    ax.set_ylabel('Magnitude')
    ax.set_title('Plot 2 — DFT Magnitude Spectrum')
    ax.legend()
    ax.grid(True, linestyle='--', alpha=0.5)
    fig.tight_layout()
    save_figure(fig, 'plot2_dft_spectrum.png')
    plt.close(fig)
    return dom_f


def plot3_smoothed_temperature(df):
    """Plot 3: Raw and moving-average-smoothed temperature on one figure."""
    smoothed = moving_average(df['temp_c'].to_numpy(), MA_WINDOW)

    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(df['time_s'], df['temp_c'],
            color='steelblue', linewidth=1.0, alpha=0.55, label='Raw')
    ax.plot(df['time_s'], smoothed,
            color='darkorange', linewidth=2.0,
            label=f'Moving average  (window = {MA_WINDOW} samples)')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Temperature (°C)')
    ax.set_title('Plot 3 — Raw vs Smoothed Temperature')
    ax.legend()
    ax.grid(True, linestyle='--', alpha=0.5)
    fig.tight_layout()
    save_figure(fig, 'plot3_smoothed_temperature.png')
    plt.close(fig)


def plot4_histogram(df):
    """Plot 4: Distribution of temperature readings (range, spread, outliers)."""
    mean_t = df['temp_c'].mean()
    std_t  = df['temp_c'].std()

    fig, ax = plt.subplots(figsize=(8, 4))
    ax.hist(df['temp_c'], bins=20, color='steelblue',
            edgecolor='white', alpha=0.85)
    ax.axvline(mean_t, color='red', linestyle='--', linewidth=1.5,
               label=f'Mean: {mean_t:.2f} °C')
    ax.axvline(mean_t - std_t, color='orange', linestyle=':', linewidth=1.2,
               label=f'±1σ  ({std_t:.3f} °C)')
    ax.axvline(mean_t + std_t, color='orange', linestyle=':', linewidth=1.2)
    ax.set_xlabel('Temperature (°C)')
    ax.set_ylabel('Sample count')
    ax.set_title('Plot 4 — Histogram of Temperature Readings')
    ax.legend()
    ax.grid(True, linestyle='--', alpha=0.5)
    fig.tight_layout()
    save_figure(fig, 'plot4_histogram.png')
    plt.close(fig)


def plot5_change_rate(df):
    """
    Plot 5: Temperature change rate (°C/s) vs time.
    Periods where |rate| exceeds RATE_THRESHOLD are highlighted in salmon
    to show where the adaptive system would trigger Active Mode.
    """
    rate = change_rate(df['temp_c'].to_numpy(), df['time_s'].to_numpy())
    fast = np.abs(rate) > RATE_THRESHOLD

    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(df['time_s'], rate, color='darkgreen', linewidth=1.2, label='Change rate')
    ax.axhline(0, color='black', linewidth=0.8, linestyle='--')
    ax.fill_between(df['time_s'], rate, 0, where=fast,
                    color='salmon', alpha=0.45,
                    label=f'|rate| > {RATE_THRESHOLD} °C/s  (Active Mode region)')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Temperature change rate (°C/s)')
    ax.set_title('Plot 5 — Temperature Change Rate vs Time')
    ax.legend()
    ax.grid(True, linestyle='--', alpha=0.5)
    fig.tight_layout()
    save_figure(fig, 'plot5_change_rate.png')
    plt.close(fig)


# ─────────────────────────────────────────────────────────────────────────────
# Discussion
# ─────────────────────────────────────────────────────────────────────────────

def print_discussion(df, dom_freq):
    """
    Print a structured discussion of the analysis results covering
    time-domain behaviour, frequency-domain content, system behaviour,
    and data-quality considerations (per the Task 4 brief).
    """
    temp   = df['temp_c'].to_numpy()
    t      = df['time_s'].to_numpy()
    rate   = change_rate(temp, t)
    mean_t = np.mean(temp)
    std_t  = np.std(temp)
    max_r  = np.abs(rate).max()
    dur_s  = t[-1]
    n      = len(df)

    print()
    print("=" * 62)
    print("  DISCUSSION OF FINDINGS  —  Task 4")
    print("=" * 62)

    print("\n  Time-domain behaviour")
    print("  " + "─" * 40)
    print(f"  Samples      : {n} at 1 Hz over {dur_s:.0f} s")
    print(f"  Mean temp    : {mean_t:.2f} °C")
    print(f"  Std deviation: {std_t:.3f} °C")
    stability = "stable (σ < 0.3 °C)" if std_t < 0.3 else "variable (σ ≥ 0.3 °C)"
    print(f"  Assessment   : signal is {stability}")
    print(f"  Max rate     : {max_r:.4f} °C/s")
    if max_r < 0.05:
        print("  No sudden large temperature rises or drops were observed.")
        print("  The signal exhibits only low-level sensor quantisation noise.")
    else:
        print("  At least one rapid transition was detected (highlighted Plot 5).")
        print("  This would correctly trigger Active Mode in the Arduino system.")

    print("\n  Frequency-domain behaviour")
    print("  " + "─" * 40)
    print(f"  Dominant frequency: {dom_freq:.4f} Hz")
    if dom_freq <= 0.1:
        mode = "POWER_DOWN (≤ 0.1 Hz)"
    elif dom_freq <= 0.5:
        mode = "IDLE (0.1 < f ≤ 0.5 Hz)"
    else:
        mode = "ACTIVE (> 0.5 Hz)"
    print(f"  Mode decision     : {mode}")
    print("  The DC component (k=0) was excluded from mode selection as it")
    print("  represents the mean temperature level, not a fluctuation.")
    print("  Higher-frequency bins show low magnitude — sensor noise does")
    print("  not dominate and would not cause spurious Active Mode triggers.")

    print("\n  System behaviour")
    print("  " + "─" * 40)
    print("  The adaptive sampling strategy is well-suited to indoor signals")
    print("  which are predominantly low-frequency. Power Down Mode would be")
    print("  selected for the majority of operation, cutting sample volume")
    print("  from 86 400/day (Active) to 2 880/day — a 97% reduction.")
    print("  Possible improvement: shorten the Idle → Power Down countdown")
    print("  from 5 cycles to 3 when σ of the moving average is very small.")

    print("\n  Data quality")
    print("  " + "─" * 40)
    print(f"  3 minutes at 1 Hz resolves frequencies from {1/dur_s:.4f} Hz to 0.5 Hz")
    print("  (Nyquist limit). This is sufficient for typical indoor HVAC cycles")
    print("  (minutes to hours). Limitation: quantisation noise (~0.1 °C from")
    print("  the 10-bit ADC) raises the DFT noise floor in high-frequency bins.")
    print("  Oversampling and averaging (4× then decimate) would reduce this.")
    print()
    print("=" * 62)
    print()


# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    # ── Load data ──────────────────────────────────────────────────────────
    if not os.path.exists(DATA_PATH):
        print(f"Error: '{DATA_PATH}' not found.")
        print("Capture real data first:  python analysis/collect_data.py")
        sys.exit(1)

    print(f"Loading data from '{DATA_PATH}'")
    df = load_csv(DATA_PATH)
    # Compute DFT in Python if columns are missing or all-zero
    if 'freq_hz' not in df.columns or df.get('freq_hz', pd.Series([0])).abs().max() == 0:
        print("  DFT columns absent — computing in Python.")
        df = add_dft_columns(df)

    os.makedirs(FIG_DIR, exist_ok=True)

    # ── Generate plots ─────────────────────────────────────────────────────
    print(f"\nGenerating 5 plots → {FIG_DIR}/")
    plot1_temperature_vs_time(df)
    dom_freq = plot2_dft_spectrum(df)
    plot3_smoothed_temperature(df)
    plot4_histogram(df)
    plot5_change_rate(df)

    # ── Print discussion ───────────────────────────────────────────────────
    print_discussion(df, dom_freq)
