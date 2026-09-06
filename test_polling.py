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
import math
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


# Candidate grids, coarsest first. The verdict is the COARSEST significant
# one: a 1 kHz stream also concentrates mod 125 us (1000 is a multiple of
# 125), but an 8 kHz stream does NOT concentrate mod 1000 (random-phase
# sources populate all 8 microframe slots, whose phases cancel) -- so
# coarsest-significant is exactly the "GCD of the deltas", jitter-robustly.
# The coarse entries catch BLE connection intervals masquerading as USB.
CANDIDATES_US = [30_000, 15_000, 11_250, 7_500, 1000, 500, 250, 125]


def grid_C(deltas, g):
    """Concentration of deltas at ZERO phase modulo grid g: 1.0 = all deltas
    are exact multiples of g, ~0 = no relation. The signed cosine (not the
    Rayleigh magnitude |R|) is essential: deltas that merely cluster in VALUE
    (250 identical 21 ms press->release pairs, human typing cadence) are
    'concentrated' modulo any grid coarser than their spread, but at an
    arbitrary phase -- the cosine cancels those, while true grid multiples
    (residues near 0) score ~1. Jitter reduces C gracefully instead of
    falling off a tolerance cliff."""
    return sum(math.cos(2 * math.pi * (d % g) / g) for d in deltas) / len(deltas)


def analyze(ts, max_delta_us):
    deltas = [b - a for a, b in zip(ts, ts[1:])]
    usable = [d for d in deltas if 0 < d <= max_delta_us]
    out = {
        "arrivals": len(ts),
        "deltas_total": len(deltas),
        "deltas_used": len(usable),
        "min_delta_us": min(deltas) if deltas else None,
    }
    n = len(usable)
    if n < 30:
        out["verdict"] = "not enough data"
        return out

    # Display histogram: every usable delta folded mod 1 ms into 125 us bins.
    bins = [0] * (FRAME_US // GRID_US)
    for d in usable:
        bins[round((d % FRAME_US) / GRID_US) % len(bins)] += 1
    out["bins_125us"] = bins

    # Zero-phase concentration per candidate grid; the significance floor
    # guards against random flukes (3.72/sqrt(n) ~ Rayleigh p=1e-6). The
    # winner is the coarsest grid scoring within 90% of the best: a 1 kHz
    # stream scores ~equally at 125/250/500/1000 us (multiples of 1000 are
    # multiples of all of them), so the coarsest of the near-ties is the true
    # quantum -- but a coarse grid with only residual cadence structure never
    # comes close to a fine grid's near-1.0 score.
    crit = max(0.15, 3.72 / math.sqrt(n))
    out["C"] = {g: round(grid_C(usable, g), 3) for g in CANDIDATES_US}
    out["C_crit"] = round(crit, 3)
    best = max(out["C"].values())
    if best < crit:
        out["verdict"] = "no polling grid visible"
        return out
    grid = next(g for g in CANDIDATES_US
                if out["C"][g] >= max(crit, 0.9 * best))
    out["grid_us"] = grid

    # Jitter estimate from the concentration at the detected grid: for
    # Gaussian stamping noise, C = exp(-2 pi^2 (sigma_delta/g)^2). sigma_delta
    # combines two timestamps, so per-stamp jitter is sigma_delta/sqrt(2).
    # A finer grid g' is detectable only while C(g') clears crit -- report
    # that floor so "no finer grid seen" isn't mistaken for "none exists".
    C = out["C"][grid]
    sigma_d = (grid / math.pi) * math.sqrt(max(math.log(1 / C), 0) / 2)
    out["stamp_jitter_us"] = round(sigma_d / math.sqrt(2), 1)
    out["finest_detectable_us"] = round(
        math.pi * sigma_d * math.sqrt(2 / math.log(1 / crit)), 1
    )

    if grid in (125, 250, 500, 1000):
        khz = 1000 // grid
        tag = " CONFIRMED" if grid == GRID_US else ""
        out["verdict"] = f"{khz} kHz polling ({grid} us grid){tag}"
    else:
        out["verdict"] = (f"arrival grid is {grid} us -- not a USB polling "
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
        print("delta mod 1 ms histogram (8 x 125 us bins; 1 kHz = peak at "
              "bin 0, 8 kHz = flat-ish, no grid = flat):")
        for i, b in enumerate(res["bins_125us"]):
            bar = "#" * round(40 * b / total)
            print(f"  {i * GRID_US:4d} us |{bar:<40}| {b}")
    if "C" in res:
        row = "   ".join(f"{g}:{c:.2f}" for g, c in res["C"].items())
        print(f"zero-phase grid concentration C (significant >= "
              f"{res['C_crit']}): {row}")
    if "stamp_jitter_us" in res:
        print(f"timestamp jitter: ~{res['stamp_jitter_us']} us per stamp; "
              f"grids finer than ~{res['finest_detectable_us']} us are "
              "invisible at this jitter")
        if res["finest_detectable_us"] > GRID_US:
            hint = ("this run had C-states pinned; the residual jitter is "
                    "elsewhere (xhci interrupt moderation?)"
                    if res.get("cstates_pinned") else
                    "rerun as root (holds /dev/cpu_dma_latency at 0 to "
                    "disable deep C-states)")
            print(f"  ^ too high to have seen a 125 us grid -- {hint}")
    if "press_release_ms" in res:
        s = res["press_release_ms"]
        print(f"press->release arrival delta (ms, firmware-timed 20 ms pair, "
              f"n={s['n']}): min {s['min']:.3f}  p50 {s['p50']:.3f}  "
              f"mean {s['mean']:.3f}  max {s['max']:.3f}   "
              "(cleanest firmware-A/B latency metric)")
    if latencies_ms:
        ls = sorted(latencies_ms)
        print(f"cmd->press latency (ms): "
              f"min {ls[0]:.2f}  p50 {ls[len(ls) // 2]:.2f}  "
              f"mean {statistics.fmean(ls):.2f}  max {ls[-1]:.2f}   "
              "(includes ~70 ms aster/subprocess overhead)")
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

    # Deep C-state exit latency (~100+ us) dominates evdev timestamp jitter
    # and can blur the 125 us grid into invisibility. Holding
    # /dev/cpu_dma_latency at 0 pins the CPUs to shallow idle states for the
    # lifetime of this fd (the cyclictest trick). Root only; warn otherwise.
    dma_latency = None
    try:
        dma_latency = open("/dev/cpu_dma_latency", "wb", buffering=0)
        dma_latency.write(struct.pack("<I", 0))
        print("C-states pinned shallow via /dev/cpu_dma_latency "
              "(low-jitter timestamps)")
    except (PermissionError, OSError):
        print("WARNING: no /dev/cpu_dma_latency access -- timestamp jitter "
              "may hide fine grids; rerun as root for a conclusive result")

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
        if dma_latency:
            dma_latency.close()

    code = None if args.manual else KEY_F24
    res = analyze(arrivals(cap.events, code), args.max_delta_ms * 1000)
    res["cstates_pinned"] = dma_latency is not None

    # Inject mode: the press->release pair is submitted 20 ms apart by the
    # firmware (k_msleep in handle_key_inject) and both ends are kernel-
    # stamped, so its arrival delta is the cleanest firmware-A/B latency
    # metric -- zero aster/subprocess noise; only the USB pickup shifts.
    if not args.manual:
        f24 = sorted((t, v) for t, c, v in cap.events if c == KEY_F24)
        pr = [(b - a) / 1000 for (a, av), (b, bv) in zip(f24, f24[1:])
              if av == 1 and bv == 0]
        if pr:
            ls = sorted(pr)
            res["press_release_ms"] = {
                "min": ls[0], "p50": ls[len(ls) // 2],
                "mean": statistics.fmean(ls), "max": ls[-1], "n": len(ls),
            }

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
