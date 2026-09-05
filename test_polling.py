#!/usr/bin/env python3
"""Verify the *actual* USB HID polling rate of the daisy keyboard.

The endpoint descriptor only advertises a polling period; this measures what
the host really does on the wire. An interrupt IN endpoint delivers a report
only when the host polls it, so every report's kernel arrival timestamp is
snapped to the bus polling grid: 1 ms frames at full speed / 1 kHz, 125 us
microframes at high speed / 8 kHz (bInterval 1). Key events therefore arrive
with inter-arrival deltas that are multiples of the grid. We fold each delta
modulo 1 ms into eight 125 us bins:

    1 kHz:  every delta is a whole number of ms  -> all mass in bin 0
    8 kHz:  deltas land on the 125 us grid       -> mass spread over all bins
    (2/4 kHz would populate bins {0,4} / {0,2,4,6})

Modes:
  --manual        you type on the keyboard (any keys) for --secs seconds.
                  Exercises the full real path: matrix scan -> debounce ->
                  HID submit -> USB poll. Grid verdict only (no ground-truth
                  press time, so no absolute latency).
  (default)       inject mode: N x `aster v1 --key-inject` (F24). Each tap =
                  press + release 20 ms apart (firmware k_msleep between the
                  two submits, factory_hid.c), a perfect short-baseline pair
                  for the grid test; also reports cmd->event latency, which
                  is where a 1 kHz -> 8 kHz A/B should show ~0.4 ms mean drop
                  (use --label / --json and diff two runs).

Physics guards baked in:
  - deltas > --max-delta-ms (default 200) are excluded from the grid test:
    the xHCI SOF clock drifts vs the kernel clock at ~ppm level, smearing
    the grid phase over long gaps (50 ppm x 1 s = 50 us -- fatal for a
    125 us grid; at 200 ms it's <= 10 us -- fine).
  - autorepeat events (value 2) are host-timer-generated, not USB arrivals:
    dropped.
  - multiple evdev events from one HID report share one timestamp: deduped.

Reading /dev/input needs root or the `input` group (falls back to
`sudo -n cat` like test_bluetooth.py). Belt-and-suspenders alternative:
capture usbmon with tshark and run the same mod-1ms analysis on URB
completion times.

`--bus bt` is a negative control: BLE arrivals ride the connection interval
(>= 7.5 ms), so no USB grid should be visible there.
"""

import argparse
import json
import statistics
import struct
import subprocess
import sys
import time
import re
import threading
from pathlib import Path

EV_KEY = 0x01
KEY_F24 = 194
EVENT_FMT = "llHHi"  # struct input_event on 64-bit
EVENT_SIZE = struct.calcsize(EVENT_FMT)

BT_NAME = "Touchpad KB"
DEFAULT_ASTER = Path.home() / "clone/aster/target/debug/aster"

GRID_US = 125          # finest hypothesis: HS microframe
FRAME_US = 1000        # FS frame / coarse hypothesis
TOL_US = 40            # |residue| tolerance: URB-completion stamping jitter


# ---------------------------------------------------------------------------
# evdev capture
# ---------------------------------------------------------------------------


def evdev_node(bus):
    """Keyboard evdev node over 'usb' or 'bt' (same logic as test_bluetooth)."""
    for name, _bus, node in keyboards():
        if bus == "usb" and _bus == "0003" and "32ac:0034" in name:
            return node
        if bus == "bt" and _bus == "0005" and BT_NAME in name:
            return node
    return None


def keyboards():
    """All evdev keyboard nodes: [('name [vid:pid]', bus_id, node), ...]."""
    out = []
    text = Path("/proc/bus/input/devices").read_text()
    for record in text.split("\n\n"):
        m = re.search(r"Handlers=([^\n]*)", record)
        if not m or "kbd" not in m.group(1).split():
            continue
        ev = next((t for t in m.group(1).split() if t.startswith("event")), None)
        name = re.search(r'Name="([^"]*)"', record)
        ids = re.search(r"Bus=(\w+) Vendor=(\w+) Product=(\w+)", record)
        if not (ev and name and ids):
            continue
        busid, vid, pid = ids.groups()
        out.append((f"{name.group(1)} [{vid}:{pid}]", busid, f"/dev/input/{ev}"))
    return out


class Capture:
    """Collects (ts_us, code, value) for EV_KEY press/release events."""

    def __init__(self, node):
        self.events = []  # (ts_us, code, value)
        self._stop = threading.Event()
        self.proc = None
        try:
            self.stream = open(node, "rb", buffering=0)
        except PermissionError:
            self.proc = subprocess.Popen(
                ["sudo", "-n", "cat", node], stdout=subprocess.PIPE
            )
            self.stream = self.proc.stdout
        self.thread = threading.Thread(target=self._watch, daemon=True)
        self.thread.start()

    def _watch(self):
        try:
            while not self._stop.is_set():
                data = self.stream.read(EVENT_SIZE)
                if not data or len(data) < EVENT_SIZE:
                    return
                sec, usec, etype, code, value = struct.unpack(EVENT_FMT, data)
                if etype == EV_KEY and value in (0, 1):  # skip autorepeat (2)
                    self.events.append((sec * 1_000_000 + usec, code, value))
        except OSError:
            pass

    def stop(self):
        self._stop.set()
        if self.proc:
            self.proc.terminate()
        try:
            self.stream.close()
        except OSError:
            pass
        self.thread.join(timeout=2)


# ---------------------------------------------------------------------------
# analysis
# ---------------------------------------------------------------------------


def arrivals(events, code=None):
    """Deduped, sorted arrival timestamps (one HID report = one arrival;
    events of one report share a timestamp)."""
    ts = sorted({t for t, c, _ in events if code is None or c == code})
    return ts


def fold(residue, grid):
    """Fold a modulo residue to [-grid/2, +grid/2)."""
    r = residue % grid
    return r - grid if r >= grid / 2 else r


def analyze(ts, max_delta_us):
    deltas = [b - a for a, b in zip(ts, ts[1:])]
    usable = [d for d in deltas if 0 < d <= max_delta_us]
    out = {
        "arrivals": len(ts),
        "deltas_total": len(deltas),
        "deltas_used": len(usable),
        "min_delta_us": min(deltas) if deltas else None,
    }
    if len(usable) < 30:
        out["verdict"] = "not enough data"
        return out

    bins = [0] * (FRAME_US // GRID_US)  # residue mod 1ms -> 8 x 125us bins
    on125 = on1000 = 0
    for d in usable:
        r125 = fold(d, GRID_US)
        r1000 = fold(d, FRAME_US)
        if abs(r125) <= TOL_US:
            on125 += 1
            bins[round((d % FRAME_US) / GRID_US) % len(bins)] += 1
        if abs(r1000) <= TOL_US:
            on1000 += 1

    n = len(usable)
    out["on_125us_grid"] = on125 / n
    out["on_1000us_grid"] = on1000 / n
    out["bins_125us"] = bins
    out["residue_rms_us"] = round(
        statistics.pstdev([fold(d, GRID_US) for d in usable]), 1
    )

    if on125 / n < 0.9:
        out["verdict"] = "no polling grid visible (jitter > tolerance?)"
        return out

    # The mod test alone can't tell a coarse grid that divides evenly into
    # 1 ms (a 15 ms BLE conn interval "passes" as 1 kHz). The actual grid is
    # the GCD of the deltas snapped to the 125 us hypothesis.
    from math import gcd
    from functools import reduce
    snapped = [round(d / GRID_US) * GRID_US for d in usable
               if abs(fold(d, GRID_US)) <= TOL_US]
    base = reduce(gcd, snapped)
    out["grid_us"] = base
    if base in (125, 250, 500, 1000):
        khz = 1000 // base
        tag = " CONFIRMED" if base == GRID_US else ""
        out["verdict"] = f"{khz} kHz polling ({base} us grid){tag}"
    else:
        out["verdict"] = (f"arrival grid is {base} us -- not a USB polling "
                          "grid (BLE conn interval?)")
    return out


def print_report(res, latencies_ms=None):
    print(f"\narrivals: {res['arrivals']}   "
          f"grid-test deltas: {res.get('deltas_used', 0)}"
          f"/{res.get('deltas_total', 0)} (rest too long / drift-unsafe)")
    if res.get("min_delta_us") is not None:
        print(f"min inter-report delta: {res['min_delta_us']} us "
              "(how close two reports actually arrived)")
    if "bins_125us" in res:
        total = sum(res["bins_125us"]) or 1
        chance = 2 * TOL_US / GRID_US  # ungridded data lands "on grid" this often
        print(f"on 125 us grid: {res['on_125us_grid']:.1%} "
              f"(chance baseline {chance:.0%}, real grid ~100%)   "
              f"on 1 ms grid: {res['on_1000us_grid']:.1%}   "
              f"residue RMS: {res['residue_rms_us']} us "
              f"(uniform/no grid = {GRID_US / 12**0.5:.0f} us)")
        print("delta mod 1 ms histogram (8 x 125 us bins; 1 kHz = bin 0 only):")
        for i, b in enumerate(res["bins_125us"]):
            bar = "#" * round(40 * b / total)
            print(f"  {i * GRID_US:4d} us |{bar:<40}| {b}")
    if latencies_ms:
        ls = sorted(latencies_ms)
        print(f"cmd->press latency (ms): "
              f"min {ls[0]:.2f}  p50 {ls[len(ls) // 2]:.2f}  "
              f"mean {statistics.fmean(ls):.2f}  max {ls[-1]:.2f}   "
              f"(A/B this between firmwares)")
    print(f"\nVERDICT: {res['verdict']}")


# ---------------------------------------------------------------------------
# modes
# ---------------------------------------------------------------------------


def run_manual(cap, secs):
    print(f"Type on the captured keyboard for {secs} s -- vary your rhythm, include some "
          "fast rollover bursts (min-delta stat). Output goes nowhere useful, "
          "so focus another window if you like; ctrl+c ends early.")
    try:
        for remaining in range(secs, 0, -1):
            print(f"\r  capturing... {remaining:3d} s, "
                  f"{len(cap.events)} events", end="", flush=True)
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    print()
    return None


def run_inject(cap, aster, count, gap):
    latencies = []
    for i in range(count):
        seen = len(cap.events)
        t0 = time.time()
        proc = subprocess.run(
            [str(aster), "v1", "--json", "--key-inject"],
            capture_output=True, text=True, timeout=30,
        )
        if proc.returncode != 0:
            sys.exit(f"aster --key-inject failed: {proc.stderr.strip()}")
        deadline = time.time() + 1.0
        while time.time() < deadline:
            new = [e for e in cap.events[seen:] if e[1] == KEY_F24 and e[2] == 1]
            if new:
                latencies.append((new[0][0] / 1e6 - t0) * 1000)
                break
            time.sleep(0.001)
        print(f"\r  inject {i + 1}/{count}", end="", flush=True)
        time.sleep(gap)
    print()
    return latencies


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("--manual", action="store_true",
                    help="you type instead of aster injecting")
    ap.add_argument("--secs", type=int, default=30,
                    help="manual capture duration (default 30)")
    ap.add_argument("-n", "--count", type=int, default=250,
                    help="inject count (default 250; 2 reports each)")
    ap.add_argument("--gap", type=float, default=0.03,
                    help="sleep between injects, s (default 0.03)")
    ap.add_argument("--bus", choices=["usb", "bt"], default="usb")
    ap.add_argument("--node", help="evdev node override (skip auto-detect)")
    ap.add_argument("--device",
                    help="capture any keyboard whose name contains this "
                    "(case-insensitive; see --list). Lets the methodology be "
                    "validated without the daisy: an external/FW16-module USB "
                    "keyboard should read '1 kHz polling'; an i8042/EC "
                    "internal keyboard is not USB, so expect no USB grid "
                    "(whatever cadence shows is the EC scan, not polling).")
    ap.add_argument("--list", action="store_true",
                    help="list keyboard evdev nodes and exit")
    ap.add_argument("--aster", type=Path, default=DEFAULT_ASTER)
    ap.add_argument("--max-delta-ms", type=float, default=200,
                    help="max delta used for the grid test (SOF-drift guard)")
    ap.add_argument("--label", default="", help="tag for the --json record")
    ap.add_argument("--json", type=Path, help="append result as a JSON line")
    args = ap.parse_args()

    if args.list:
        for name, busid, node in keyboards():
            print(f"{node}  bus {busid}  {name}")
        return

    if args.device:
        match = next((k for k in keyboards()
                      if args.device.lower() in k[0].lower()), None)
        if not match:
            sys.exit(f"no keyboard matching {args.device!r} (see --list)")
        node, desc = match[2], match[0]
        if not args.manual:
            sys.exit("--device implies --manual (key-inject only exists on daisy)")
    else:
        node = args.node or evdev_node(args.bus)
        desc = args.bus
        if not node:
            sys.exit(f"no {args.bus} keyboard evdev node found -- device "
                     "plugged in? (--list shows all keyboards, --device picks one)")
    print(f"capturing {node} ({desc})")

    cap = Capture(node)
    time.sleep(0.3)
    try:
        if args.manual:
            latencies = run_manual(cap, args.secs)
        else:
            if not args.aster.exists():
                sys.exit(f"aster not found at {args.aster} (--aster or --manual)")
            latencies = run_inject(cap, args.aster, args.count, args.gap)
        time.sleep(0.3)
    finally:
        cap.stop()

    code = None if args.manual else KEY_F24
    res = analyze(arrivals(cap.events, code), args.max_delta_ms * 1000)
    print_report(res, latencies)

    if args.json:
        record = {
            "label": args.label, "mode": "manual" if args.manual else "inject",
            "bus": args.bus, "time": time.strftime("%Y-%m-%dT%H:%M:%S"),
            **res,
        }
        if latencies:
            record["latency_ms"] = {
                "min": min(latencies), "mean": statistics.fmean(latencies),
                "max": max(latencies), "n": len(latencies),
            }
        with args.json.open("a") as f:
            f.write(json.dumps(record) + "\n")
        print(f"appended to {args.json}")

    # 0 = conclusive verdict (any rate); 1 = couldn't determine the grid.
    conclusive = "kHz polling" in res.get("verdict", "")
    sys.exit(0 if conclusive else 1)


if __name__ == "__main__":
    main()
