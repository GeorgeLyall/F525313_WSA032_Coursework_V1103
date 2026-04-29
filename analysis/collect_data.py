"""
collect_data.py

Captures CSV data from the Arduino temperature monitoring sketch
(temperature_optimisation.ino) over Serial and saves it to
data/temperature_data.csv for analysis by temperature_analysis.py.

The sketch delimits each data block with:
    --- DATA START ---
    Time(ms),Temperature(C),Frequency(Hz),Magnitude
    ...rows...
    --- DATA END ---

Usage (from repo root):
    python analysis/collect_data.py
    python analysis/collect_data.py --port COM3 --duration 3
    python analysis/collect_data.py --port COM3 --cycles 5
    python analysis/collect_data.py --port COM3 --duration 3 --cycles 5

Arguments:
    --port      Serial port the Arduino is on (e.g. COM3, /dev/ttyUSB0).
                If omitted the script lists available ports and asks.
    --duration  How many minutes to collect data for.  Collection stops
                when this time has elapsed (measured from the first data
                block received).  Default: unlimited.
    --cycles    Maximum number of complete data blocks to capture.
                Default: unlimited.
                If both --duration and --cycles are given, whichever
                limit is reached first stops collection.
                If neither is given, collection runs until Ctrl+C.
    --out       Output CSV path (default: data/temperature_data.csv).
    --baud      Baud rate (default: 9600, must match Serial.begin() call).
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
DEFAULT_BAUD      = 9600
DEFAULT_OUT       = os.path.join('data', 'temperature_data.csv')
MARKER_START      = '--- DATA START ---'
MARKER_END        = '--- DATA END ---'
BLOCK_TIMEOUT_S   = 300  # max seconds to wait for a single block to complete


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

def capture(port, baud, max_cycles, duration_s):
    """
    Open the serial port and collect data until max_cycles complete blocks
    have been received OR duration_s seconds have elapsed since the first
    block started — whichever comes first.  If both are None, runs until
    the user presses Ctrl+C.

    Returns (header, rows) where header is the CSV header string and rows
    is a list of data-row strings.
    """
    print(f"Opening {port} at {baud} baud …")
    try:
        # Open with DTR held low so the Arduino does not reset on connect
        ser = serial.Serial()
        ser.port     = port
        ser.baudrate = baud
        ser.timeout  = 1
        ser.dtr      = False
        ser.open()
    except serial.SerialException as exc:
        print(f"Could not open port: {exc}")
        sys.exit(1)

    ser.reset_input_buffer()

    limit_str = []
    if duration_s is not None:
        limit_str.append(f"{duration_s / 60:.4g} min")
    if max_cycles is not None:
        limit_str.append(f"{max_cycles} cycle(s)")
    print("Collecting" + (f" for {' / '.join(limit_str)}" if limit_str else " until Ctrl+C")
          + " …  (press Ctrl+C to stop early)\n")

    header         = None
    rows           = []
    in_block       = False
    cycles         = 0
    collection_end = None          # set when the first block starts
    block_deadline = None          # per-block timeout

    try:
        while True:
            now = time.time()

            # Duration check — only active once first block has started
            if collection_end is not None and now > collection_end:
                print(f"\nDuration reached — stopping after {cycles} complete block(s).")
                break

            # Per-block stall guard
            if block_deadline is not None and now > block_deadline:
                print("\nTimeout waiting for block to complete.")
                ser.close()
                sys.exit(1)

            raw = ser.readline()
            if not raw:
                continue

            try:
                line = raw.decode('ascii', errors='replace').strip()
            except Exception:
                continue

            # Echo status lines so the user can see the sketch is running
            if not in_block:
                print(line)

            if MARKER_START in line:
                in_block       = True
                rows_this_block = []
                block_deadline  = time.time() + BLOCK_TIMEOUT_S
                # Start the duration clock on the very first block
                if collection_end is None and duration_s is not None:
                    collection_end = time.time() + duration_s
                continue

            if MARKER_END in line:
                in_block       = False
                block_deadline = None
                cycles        += 1
                rows.extend(rows_this_block)
                print(f"\n  Block {cycles} captured ({len(rows_this_block)} rows"
                      + (f", {int(collection_end - time.time())}s remaining"
                         if collection_end is not None else "")
                      + ")")
                if max_cycles is not None and cycles >= max_cycles:
                    print(f"Cycle limit reached ({max_cycles}).")
                    break
                continue

            if in_block:
                if line.startswith('Time'):
                    if header is None:
                        header = line   # capture once; skip duplicates from later cycles
                elif header and line:
                    rows_this_block.append(line)

    except KeyboardInterrupt:
        print(f"\nStopped by user after {cycles} complete block(s).")

    ser.close()
    return header, rows


# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--port',     default=None,
                        help='Serial port (e.g. COM3)')
    parser.add_argument('--baud',     type=int, default=DEFAULT_BAUD,
                        help='Baud rate (default: 9600)')
    parser.add_argument('--duration', type=float, default=None,
                        help='Collection duration in minutes')
    parser.add_argument('--cycles',   type=int, default=None,
                        help='Maximum number of cycles to capture')
    parser.add_argument('--out',      default=DEFAULT_OUT,
                        help='Output CSV path (default: data/temperature_data.csv)')
    args = parser.parse_args()

    port       = args.port or pick_port()
    duration_s = args.duration * 60 if args.duration is not None else None

    header, rows = capture(port, args.baud, args.cycles, duration_s)

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
