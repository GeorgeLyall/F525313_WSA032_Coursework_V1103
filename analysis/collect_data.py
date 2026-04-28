"""
collect_data.py

Captures one cycle of CSV data from the Arduino temperature monitoring
sketch (temperature_optimisation.ino) over Serial and saves it to
data/temperature_data.csv for analysis by temperature_analysis.py.

The sketch delimits each data block with:
    --- DATA START ---
    Time(ms),Temperature(C),Frequency(Hz),Magnitude
    ...rows...
    --- DATA END ---

This script waits for the first complete block then exits.

Usage (from repo root):
    python analysis/collect_data.py
    python analysis/collect_data.py --port COM3
    python analysis/collect_data.py --port COM3 --cycles 3

Arguments:
    --port    Serial port the Arduino is on (e.g. COM3, /dev/ttyUSB0).
              If omitted the script lists available ports and asks.
    --cycles  Number of complete data blocks to capture before saving
              (default 1).  Multiple cycles are concatenated.
    --out     Output CSV path (default: data/temperature_data.csv).
    --baud    Baud rate (default: 9600, must match Serial.begin() call).
"""

import argparse
import os
import sys
import time

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("pyserial is not installed.  Run:  pip install pyserial")
    sys.exit(1)


# ── Defaults ──────────────────────────────────────────────────────────────────
DEFAULT_BAUD    = 9600
DEFAULT_OUT     = os.path.join('data', 'temperature_data.csv')
MARKER_START    = '--- DATA START ---'
MARKER_END      = '--- DATA END ---'
TIMEOUT_SECONDS = 300  # 5 minutes — one full 60-second cycle + DFT overhead


# ─────────────────────────────────────────────────────────────────────────────
# Port helpers
# ─────────────────────────────────────────────────────────────────────────────

def list_ports():
    ports = serial.tools.list_ports.comports()
    return sorted(ports, key=lambda p: p.device)


def pick_port():
    """Print available ports and prompt the user to choose one."""
    ports = list_ports()
    if not ports:
        print("No serial ports found.  Is the Arduino connected?")
        sys.exit(1)
    if len(ports) == 1:
        print(f"Auto-selected only available port: {ports[0].device}")
        return ports[0].device
    print("Available serial ports:")
    for i, p in enumerate(ports):
        print(f"  [{i}] {p.device}  —  {p.description}")
    while True:
        raw = input("Enter port number or name: ").strip()
        if raw.isdigit() and 0 <= int(raw) < len(ports):
            return ports[int(raw)].device
        if any(raw == p.device for p in ports):
            return raw
        print("  Invalid choice, try again.")


# ─────────────────────────────────────────────────────────────────────────────
# Capture
# ─────────────────────────────────────────────────────────────────────────────

def capture_cycles(port, baud, n_cycles):
    """
    Open the serial port and collect n_cycles complete CSV blocks.
    Returns a list of strings: the CSV header followed by all data rows.
    """
    print(f"Opening {port} at {baud} baud …")
    try:
        ser = serial.Serial(port, baud, timeout=1)
    except serial.SerialException as exc:
        print(f"Could not open port: {exc}")
        sys.exit(1)

    # Give the Arduino time to reset after DTR toggles on connect
    time.sleep(2)
    ser.reset_input_buffer()
    print("Waiting for Arduino data …  (press Ctrl+C to cancel)\n")

    header    = None
    rows      = []
    in_block  = False
    cycles    = 0
    deadline  = time.time() + TIMEOUT_SECONDS

    try:
        while cycles < n_cycles:
            if time.time() > deadline:
                print("\nTimeout — no complete data block received.")
                ser.close()
                sys.exit(1)

            raw = ser.readline()
            if not raw:
                continue

            try:
                line = raw.decode('ascii', errors='replace').strip()
            except Exception:
                continue

            # Echo non-data lines so the user can see the sketch is running
            if not in_block:
                print(line)

            if MARKER_START in line:
                in_block = True
                rows_this_block = []
                continue

            if MARKER_END in line:
                in_block = False
                cycles += 1
                rows.extend(rows_this_block)
                print(f"\n  Block {cycles}/{n_cycles} captured "
                      f"({len(rows_this_block)} data rows)")
                deadline = time.time() + TIMEOUT_SECONDS  # reset for next block
                continue

            if in_block:
                if header is None and line.startswith('Time'):
                    header = line          # capture CSV header once
                elif header and line:
                    rows_this_block.append(line)

    except KeyboardInterrupt:
        print("\nCancelled by user.")
        ser.close()
        sys.exit(0)

    ser.close()
    return header, rows


# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--port',   default=None,         help='Serial port (e.g. COM3)')
    parser.add_argument('--baud',   type=int, default=DEFAULT_BAUD, help='Baud rate')
    parser.add_argument('--cycles', type=int, default=1,  help='Cycles to capture')
    parser.add_argument('--out',    default=DEFAULT_OUT,  help='Output CSV path')
    args = parser.parse_args()

    port = args.port or pick_port()
    header, rows = capture_cycles(port, args.baud, args.cycles)

    if not rows:
        print("No data rows captured — nothing saved.")
        sys.exit(1)

    os.makedirs(os.path.dirname(args.out) or '.', exist_ok=True)
    with open(args.out, 'w') as f:
        f.write(header + '\n')
        f.write('\n'.join(rows) + '\n')

    print(f"\nSaved {len(rows)} rows to '{args.out}'")
    print("Run  python analysis/temperature_analysis.py  to generate plots.")


if __name__ == '__main__':
    main()
