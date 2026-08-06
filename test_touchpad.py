#!/usr/bin/env python3
"""Daisy touchpad end-to-end diagnostic (host side).

Walks the whole chain layer by layer and tells you where it breaks:

  physical TP --I2C--> hid_touchpad.c --cb--> hid_passthrough_{usb_next,ble}.c
      --USB iface / HOGP--> kernel hid-multitouch --> evdev --> libinput

Phases:
  1 enum      USB device + HID interfaces + kernel drivers + report
              descriptor sanity. No interaction, no root.
  2 features  GET_FEATURE round-trips over hidraw for every feature report
              the firmware proxies (exercises USB control transfer ->
              get_report_cb -> I2C -> touchpad), with timing stats.
              Optionally SET input mode = 3 (PTP) and read it back
              (exercises set_report_cb; idempotent, hid-multitouch sets
              the same value at probe).
  3 input     Interactive: you move/tap fingers on the pad while the raw
              hidraw stream and all evdev nodes are captured in parallel.
              Reports: rate, inter-report jitter, PTP scan-time vs host
              clock drift, dropped-frame estimate, X/Y coverage, contact
              count, and whether hid-multitouch turned it into evdev events.
  4 ble       (--ble) repeat 2+3 against the Bluetooth HOGP hid device,
              so USB and BLE timings can be compared.

The end summary maps failures to the most likely broken layer, with
pointers into the firmware sources and host-side references
(~/clone/linux/drivers/hid/hid-multitouch.c, hid-tools' hid-recorder,
libinput debug-events).

Typical usage:
  sudo ./test_touchpad.py                # full USB run (asks you to touch)
  sudo ./test_touchpad.py --quick        # enum + features only, no interaction
  sudo ./test_touchpad.py --ble          # also test the BLE transport
  sudo ./test_touchpad.py --watch        # live parsed report stream (^C ends)
  sudo ./test_touchpad.py --record m.jsonl --json result.json

hidraw access needs root (the script re-execs itself with sudo if needed;
--no-sudo disables that). Enumeration alone works as a plain user.
"""

import argparse
import fcntl
import json
import os
import re
import select
import statistics
import struct
import subprocess
import sys
import time

VID, PID = 0x32AC, 0x0034
BUS_USB, BUS_BT = 0x0003, 0x0005
BT_NAME_HINT = "Touchpad KB"

MOUSE_ID, PTP_ID = 0x01, 0x04
MOUSE_PAYLOAD, PTP_PAYLOAD = 8, 29
X_MAX, Y_MAX = 796, 998  # logical maxima from the report descriptor
SCAN_TIME_UNIT_S = 100e-6  # PTP scan time: unit exponent -4, seconds
NUM_FINGERS = 5

# Feature reports the firmware proxies (daisy-touchpad.dtsi feature-report-ids),
# with the payload size (excl. report id) from the report descriptor and
# whether a working GET is required for a healthy PTP touchpad.
FEATURES = {
    0x02: ("device capabilities (contact max)", 1, True),
    0x03: ("input mode", 1, True),
    0x05: ("selective reporting (surface/button switch)", 1, False),
    0x06: ("digitizer 0x59 (pad type)", 1, False),
    0x07: ("latency mode", 1, False),
    0x0A: ("PTPHQA certification blob", 256, True),
    0x0B: ("vendor 0x0B", 1, False),
    0x41: ("vendor blob 0x41", 256, False),
    0x42: ("vendor 0x42", 3, False),
    0x43: ("vendor 0x43", 3, False),
}
INPUT_MODE_PTP = 3  # digitizer Input Mode value for "Windows precision touchpad"

# evdev constants (linux/input-event-codes.h)
EV_SYN, EV_KEY, EV_REL, EV_ABS, EV_MSC = 0x00, 0x01, 0x02, 0x03, 0x04
REL_X, REL_Y = 0x00, 0x01
ABS_X, ABS_Y = 0x00, 0x01
ABS_MT_SLOT = 0x2F
ABS_MT_POSITION_X, ABS_MT_POSITION_Y = 0x35, 0x36
ABS_MT_TRACKING_ID = 0x39
BTN_LEFT, BTN_RIGHT = 0x110, 0x111
BTN_TOUCH, BTN_TOOL_FINGER, BTN_TOOL_DOUBLETAP = 0x14A, 0x145, 0x14D
EVDEV_EVENT_FMT = "llHHi"
EVDEV_EVENT_SIZE = struct.calcsize(EVDEV_EVENT_FMT)


# ---------------------------------------------------------------- ioctl glue

def _ioc(direction, nr, size):
    return (direction << 30) | (size << 16) | (ord("H") << 8) | nr


def hidraw_get_feature(fd, report_id, payload_len):
    """Returns (bytes_payload, seconds) or raises OSError."""
    buf = bytearray(payload_len + 1)
    buf[0] = report_id
    t0 = time.monotonic()
    n = fcntl.ioctl(fd, _ioc(3, 0x07, len(buf)), buf, True)  # HIDIOCGFEATURE
    dt = time.monotonic() - t0
    return bytes(buf[1:n]), dt


def hidraw_set_feature(fd, report_id, payload):
    buf = bytearray([report_id]) + bytearray(payload)
    t0 = time.monotonic()
    fcntl.ioctl(fd, _ioc(3, 0x06, len(buf)), buf, True)  # HIDIOCSFEATURE
    return time.monotonic() - t0


# ---------------------------------------------------------------- discovery

class HidDev:
    """One /sys/bus/hid/devices entry."""

    def __init__(self, syspath):
        self.syspath = syspath
        base = os.path.basename(syspath)
        m = re.match(r"([0-9A-Fa-f]{4}):([0-9A-Fa-f]{4}):([0-9A-Fa-f]{4})\.", base)
        self.bus, self.vid, self.pid = (int(g, 16) for g in m.groups())
        self.hid_id = base
        self.name = self._uevent().get("HID_NAME", "?")
        self.driver = None
        drv = os.path.join(syspath, "driver")
        if os.path.islink(drv):
            self.driver = os.path.basename(os.readlink(drv))
        try:
            with open(os.path.join(syspath, "report_descriptor"), "rb") as f:
                self.rdesc = f.read()
        except OSError:
            self.rdesc = b""
        self.hidraw = None
        hr = os.path.join(syspath, "hidraw")
        if os.path.isdir(hr):
            nodes = sorted(os.listdir(hr))
            if nodes:
                self.hidraw = "/dev/" + nodes[0]
        self.event_nodes = []  # [(path, name)]
        inp = os.path.join(syspath, "input")
        if os.path.isdir(inp):
            for i in sorted(os.listdir(inp)):
                iname = "?"
                try:
                    with open(os.path.join(inp, i, "name")) as f:
                        iname = f.read().strip()
                except OSError:
                    pass
                for ev in sorted(os.listdir(os.path.join(inp, i))):
                    if ev.startswith("event"):
                        self.event_nodes.append(("/dev/input/" + ev, iname))
        self.usb_iface = None
        try:
            real = os.path.realpath(syspath)
            with open(os.path.join(real, "..", "bInterfaceNumber")) as f:
                self.usb_iface = int(f.read().strip(), 16)
        except OSError:
            pass

    def _uevent(self):
        out = {}
        try:
            with open(os.path.join(self.syspath, "uevent")) as f:
                for line in f:
                    if "=" in line:
                        k, v = line.strip().split("=", 1)
                        out[k] = v
        except OSError:
            pass
        return out

    @property
    def kind(self):
        if b"\x05\x0d\x09\x05" in self.rdesc:  # Usage Page Digitizer, Usage Touch Pad
            return "touchpad"
        if b"\x05\x01\x09\x06" in self.rdesc:  # Usage Page GD, Usage Keyboard
            return "keyboard"
        return "vendor"

    def __repr__(self):
        return (f"{self.hid_id} [{self.kind}] driver={self.driver} "
                f"hidraw={self.hidraw} iface={self.usb_iface}")


def find_daisy_hid(bus):
    devs = []
    for p in sorted(os.listdir("/sys/bus/hid/devices")):
        d = HidDev(os.path.join("/sys/bus/hid/devices", p))
        if d.bus != bus:
            continue
        if (d.vid, d.pid) == (VID, PID) or (bus == BUS_BT and BT_NAME_HINT in d.name):
            devs.append(d)
    return devs


def usb_sysfs_info():
    """busnum/devnum/bcdDevice of the daisy USB device, or None."""
    for p in sorted(os.listdir("/sys/bus/usb/devices")):
        d = os.path.join("/sys/bus/usb/devices", p)
        try:
            with open(os.path.join(d, "idVendor")) as f:
                v = f.read().strip()
            with open(os.path.join(d, "idProduct")) as f:
                pr = f.read().strip()
        except OSError:
            continue
        if (v, pr) == ("%04x" % VID, "%04x" % PID):
            info = {"syspath": d}
            for attr in ("busnum", "devnum", "bcdDevice", "product", "serial"):
                try:
                    with open(os.path.join(d, attr)) as f:
                        info[attr] = f.read().strip()
                except OSError:
                    pass
            return info
    return None


# ---------------------------------------------------------------- reporting

class C:
    G, R, Y, B, DIM, OFF = "\033[32m", "\033[31m", "\033[33m", "\033[36m", "\033[2m", "\033[0m"
    on = sys.stdout.isatty()

    @classmethod
    def c(cls, code, s):
        return f"{code}{s}{cls.OFF}" if cls.on else s


class Report:
    def __init__(self):
        self.checks = []   # (phase, name, status, detail)  status: pass/fail/warn/skip
        self.data = {}     # free-form measurements for --json
        self.phase = "?"

    def check(self, name, ok, detail="", warn_only=False, fail_detail=None):
        if not ok and fail_detail is not None:
            detail = fail_detail if not detail else f"{detail} — {fail_detail}"
        elif ok and fail_detail is not None:
            pass  # fail_detail is only shown on failure
        status = "pass" if ok else ("warn" if warn_only else "fail")
        self.checks.append((self.phase, name, status, detail))
        mark = {"pass": C.c(C.G, "PASS"), "fail": C.c(C.R, "FAIL"),
                "warn": C.c(C.Y, "WARN")}[status]
        print(f"  [{mark}] {name}" + (f" {C.c(C.DIM, '— ' + detail)}" if detail else ""))
        return ok

    def skip(self, name, why):
        self.checks.append((self.phase, name, "skip", why))
        print(f"  [{C.c(C.DIM, 'SKIP')}] {name} — {why}")

    def info(self, msg):
        print(f"  {C.c(C.DIM, msg)}")

    def banner(self, title):
        self.phase = title
        print(f"\n{C.c(C.B, '━━ ' + title + ' ' + '━' * max(0, 60 - len(title)))}")

    def failed(self, phase=None):
        return [c for c in self.checks
                if c[2] == "fail" and (phase is None or c[0] == phase)]


def prompt(msg, interactive=True):
    print(f"\n  {C.c(C.Y, '>>> ' + msg)}")
    if interactive:
        try:
            input("      Press Enter when ready (or type 's' + Enter to skip)... ") \
                .strip().lower()
        except EOFError:
            return True
    return True


def ask(msg):
    print(f"\n  {C.c(C.Y, '>>> ' + msg)}")
    try:
        return input("      Press Enter when ready, 's' to skip: ").strip().lower() != "s"
    except EOFError:
        return True


def fmt_ms(seconds):
    return f"{seconds * 1e3:.2f}ms"


def stats_line(vals_ms):
    if not vals_ms:
        return "n/a"
    return (f"min {min(vals_ms):.2f} / med {statistics.median(vals_ms):.2f} / "
            f"avg {statistics.mean(vals_ms):.2f} / max {max(vals_ms):.2f} ms")


# ---------------------------------------------------------------- parsing

def s8(b):
    return b - 256 if b > 127 else b


def parse_mouse(p):
    if len(p) < 5:
        return None
    return {"buttons": p[0] & 0x3, "dx": s8(p[1]), "dy": s8(p[2]),
            "wheel": s8(p[3]), "pan": s8(p[4])}


def parse_ptp(p):
    if len(p) < PTP_PAYLOAD:
        return None
    fingers = []
    for i in range(NUM_FINGERS):
        b = p[i * 5]
        fingers.append({
            "confidence": b & 1, "tip": (b >> 1) & 1, "cid": b >> 4,
            "x": int.from_bytes(p[i * 5 + 1:i * 5 + 3], "little"),
            "y": int.from_bytes(p[i * 5 + 3:i * 5 + 5], "little"),
        })
    return {"fingers": fingers, "contact_count": p[25], "buttons": p[26] & 0x7,
            "scan_time": int.from_bytes(p[27:29], "little")}


# ---------------------------------------------------------------- capture

class Capture:
    """Simultaneously read the TP hidraw node + all its evdev nodes."""

    def __init__(self, tp, record_fh=None, tag="", extra_nodes=None):
        self.tp = tp
        self.record_fh = record_fh
        self.tag = tag
        self.extra_nodes = extra_nodes or []  # [(path, label)] e.g. keyboard evdev
        self.hid_events = []    # (t_mono, report_id, payload bytes)
        self.evdev_events = {}  # node -> [(t_mono, type, code, value)]
        self.t_start = self.t_end = None
        self._fds = {}

    def __enter__(self):
        fd = os.open(self.tp.hidraw, os.O_RDONLY | os.O_NONBLOCK)
        self._fds[fd] = ("hidraw", self.tp.hidraw)
        for node, _name in list(self.tp.event_nodes) + list(self.extra_nodes):
            try:
                efd = os.open(node, os.O_RDONLY | os.O_NONBLOCK)
                self._fds[efd] = ("evdev", node)
                self.evdev_events[node] = []
            except OSError as e:
                print(f"  {C.c(C.DIM, f'(cannot open {node}: {e.strerror})')}")
        self._drain()
        return self

    def __exit__(self, *exc):
        for fd in self._fds:
            os.close(fd)

    def _drain(self):
        for fd in self._fds:
            try:
                while os.read(fd, 4096):
                    pass
            except OSError:
                pass

    def run(self, duration, progress=True):
        self.t_start = time.monotonic()
        end = self.t_start + duration
        next_tick = time.monotonic() + 1
        while True:
            now = time.monotonic()
            if now >= end:
                break
            if progress and now >= next_tick:
                remaining = int(end - now + 0.999)
                print(f"\r      capturing... {remaining:2d}s left, "
                      f"{len(self.hid_events)} raw reports", end="", flush=True)
                next_tick = now + 1
            r, _, _ = select.select(list(self._fds), [], [], min(0.05, end - now))
            for fd in r:
                kind, node = self._fds[fd]
                try:
                    data = os.read(fd, 4096)
                except OSError:
                    continue
                t = time.monotonic()
                if kind == "hidraw":
                    if data:
                        self.hid_events.append((t, data[0], data[1:]))
                        if self.record_fh:
                            self.record_fh.write(json.dumps({
                                "t": t, "src": f"hidraw{self.tag}",
                                "id": data[0], "data": data[1:].hex()}) + "\n")
                else:
                    for off in range(0, len(data) - EVDEV_EVENT_SIZE + 1,
                                     EVDEV_EVENT_SIZE):
                        _s, _u, etype, code, value = struct.unpack_from(
                            EVDEV_EVENT_FMT, data, off)
                        self.evdev_events[node].append((t, etype, code, value))
        self.t_end = time.monotonic()
        if progress:
            print("\r" + " " * 60 + "\r", end="")


def unwrap_scan_times(raws):
    out, off, prev = [], 0, None
    for v in raws:
        if prev is not None and v < prev:
            off += 0x10000
        out.append(v + off)
        prev = v
    return out


def analyze_motion(cap, rep, res_prefix):
    """Stats + checks for a continuous one-finger motion capture."""
    by_id = {}
    for _t, rid, _p in cap.hid_events:
        by_id[rid] = by_id.get(rid, 0) + 1
    rep.info(f"raw reports by id: " +
             (", ".join(f"0x{i:02x}×{n}" for i, n in sorted(by_id.items())) or "none"))
    out = {"by_id": by_id}

    ptp = [(t, parse_ptp(p)) for t, rid, p in cap.hid_events if rid == PTP_ID]
    ptp = [(t, f) for t, f in ptp if f]
    mouse_n = by_id.get(MOUSE_ID, 0)

    got_any = rep.check("raw input reports arrive on hidraw", bool(cap.hid_events),
                        f"{len(cap.hid_events)} reports",
                        fail_detail="firmware interrupt path dead (DR gpio / "
                        "input cb / hid_ready / endpoint routing?)")
    if got_any:
        rep.check("PTP (report id 0x04) frames present", bool(ptp),
                  f"only mouse-mode reports (id 0x01 ×{mouse_n}) — input mode "
                  f"never set to {INPUT_MODE_PTP}?" if mouse_n and not ptp else
                  f"{len(ptp)} frames")

    if len(ptp) >= 10:
        # host-side inter-arrival
        times = [t for t, _ in ptp]
        span = times[-1] - times[0]
        all_deltas = [b - a for a, b in zip(times, times[1:])]
        cont = [d for d in all_deltas if d < 0.1]  # "continuous stream" deltas
        gaps = [d for d in all_deltas if d >= 0.1]
        overall = (len(ptp) - 1) / span if span > 0 else 0.0
        burst = 1.0 / statistics.median(cont) if cont else overall
        jitter = [d * 1e3 for d in (cont or all_deltas)]
        rep.info(f"host inter-arrival: {stats_line(jitter)}  "
                 f"({overall:.1f} Hz overall, ~{burst:.0f} Hz within bursts)")
        if gaps:
            rep.info(f"gaps >=100ms between reports: {len(gaps)} "
                     f"(longest {max(gaps) * 1e3:.0f}ms) — stream is NOT continuous")
        out["rate_hz"] = round(burst, 1)
        out["rate_overall_hz"] = round(overall, 1)
        out["host_gaps_100ms"] = len(gaps)
        if jitter:
            out["host_interarrival_ms"] = {
                "min": min(jitter), "med": statistics.median(jitter),
                "max": max(jitter)}
        rep.check("report rate >= 60 Hz", overall >= 60,
                  f"{overall:.1f} Hz overall ({burst:.0f} Hz within bursts)",
                  warn_only=True)

        # device-side timeline: scan time (100 us units)
        scans = unwrap_scan_times([f["scan_time"] for _, f in ptp])
        sdeltas = [(b - a) * SCAN_TIME_UNIT_S for a, b in zip(scans, scans[1:])]
        pos = [d for d in sdeltas if d > 0]
        if pos:
            med = statistics.median(pos)
            drops = sum(1 for d in pos if d > 1.75 * med)
            dups = sum(1 for d in sdeltas if d == 0)
            rep.info(f"device scan-time delta: med {med*1e3:.2f}ms; "
                     f"gaps>1.75×med: {drops}, duplicates: {dups}")
            out["scan_med_ms"] = med * 1e3
            out["frame_gaps"] = drops
            rep.check("dropped-frame estimate < 2%",
                      drops <= max(1, len(pos) * 0.02),
                      f"{drops}/{len(pos)} scan-time gaps", warn_only=True)
            # compare device pacing vs host arrival pacing
            host_med = statistics.median(all_deltas) if all_deltas else 0
            if host_med > 0.05:
                if med < 0.5 * host_med:
                    rep.info(C.c(C.Y,
                        f"pad stamps frames {med*1e3:.0f}ms apart but they reach "
                        f"the host {host_med*1e3:.0f}ms apart -> frames are "
                        f"delayed/queued between pad and host (firmware submit "
                        f"path: TX semaphore / usbd)"))
                else:
                    rep.info(C.c(C.Y,
                        f"scan-time gaps match host gaps ({med*1e3:.0f}ms vs "
                        f"{host_med*1e3:.0f}ms): consecutive *received* frames "
                        f"are genuinely that far apart -> pad producing slowly, "
                        f"I2C reads stalling, or intermediate frames dropped "
                        f"before submit (check RTT: 'TX buffer still in flight')"))
            # drift: device clock span vs host clock span
            span_dev = (scans[-1] - scans[0]) * SCAN_TIME_UNIT_S
            span_host = times[-1] - times[0]
            if span_host > 1:
                drift = (span_dev - span_host) / span_host * 100
                rep.info(f"device vs host clock span: {span_dev:.2f}s vs "
                         f"{span_host:.2f}s ({drift:+.2f}%)")
                out["clock_drift_pct"] = round(drift, 3)

        xs = [f2["x"] for _, f in ptp for f2 in f["fingers"] if f2["tip"]]
        ys = [f2["y"] for _, f in ptp for f2 in f["fingers"] if f2["tip"]]
        if xs:
            rep.info(f"coverage: X {min(xs)}..{max(xs)} (max {X_MAX}), "
                     f"Y {min(ys)}..{max(ys)} (max {Y_MAX})")
            out["x_range"] = [min(xs), max(xs)]
            out["y_range"] = [min(ys), max(ys)]
            rep.check("coordinates inside logical range",
                      max(xs) <= X_MAX and max(ys) <= Y_MAX,
                      f"X<= {max(xs)}, Y<= {max(ys)}")
        maxcc = max(f["contact_count"] for _, f in ptp)
        out["max_contact_count"] = maxcc

    # evdev side
    total_ev = {n: len(v) for n, v in cap.evdev_events.items()}
    mt_nodes = [n for n, evs in cap.evdev_events.items()
                if any(e[1] == EV_ABS and e[2] in (ABS_MT_POSITION_X, ABS_X)
                       for e in evs)]
    rep.info("evdev: " + (", ".join(f"{n}: {c} events" for n, c in total_ev.items())
                          or "no nodes opened"))
    if cap.evdev_events:
        rep.check("kernel driver emits evdev motion (hid-multitouch -> input)",
                  bool(mt_nodes),
                  "raw reports arrive but no ABS events — host-side parsing "
                  "issue (check hid-recorder + hid-multitouch quirks)"
                  if cap.hid_events and not mt_nodes else ", ".join(mt_nodes))
    out["evdev_counts"] = total_ev
    rep.data[res_prefix] = out
    return out


def analyze_tap(cap, rep, res_prefix):
    ptp = [(t, parse_ptp(p)) for t, rid, p in cap.hid_events if rid == PTP_ID]
    ptp = [(t, f) for t, f in ptp if f]
    touches, down_t = [], None
    for t, f in ptp:
        tip = any(fg["tip"] for fg in f["fingers"])
        if tip and down_t is None:
            down_t = t
        elif not tip and down_t is not None:
            touches.append((t - down_t) * 1e3)
            down_t = None
    rep.check("tap produced tip-switch down/up", bool(touches),
              f"{len(touches)} touch(es), duration(s): "
              + ", ".join(f"{d:.0f}ms" for d in touches[:5]) if touches else
              "no tip transitions seen in PTP frames")
    btn = [e for evs in cap.evdev_events.values() for e in evs
           if e[1] == EV_KEY and e[2] == BTN_TOUCH]
    rep.check("evdev BTN_TOUCH followed", bool(btn),
              f"{len(btn)} transitions" if btn else "none seen", warn_only=True)
    rep.data[res_prefix] = {"touch_durations_ms": touches,
                            "btn_touch_events": len(btn)}


def analyze_interference(cap, rep, res_prefix, kb_nodes):
    """Correlate keyboard key events with gaps/death in the TP stream."""
    tp_times = [t for t, _rid, _p in cap.hid_events]
    presses = [t for node in kb_nodes for (t, et, code, val)
               in cap.evdev_events.get(node, [])
               if et == EV_KEY and val == 1]
    rep.info(f"{len(tp_times)} TP reports, {len(presses)} key presses captured")
    out = {"tp_reports": len(tp_times), "key_presses": len(presses)}

    if not presses:
        rep.skip("keyboard/touchpad interference",
                 "no key presses captured — tap the *daisy* keyboard during "
                 "the capture (and check the keyboard evdev node was openable)")
        rep.data[res_prefix] = out
        return
    if len(tp_times) < 10:
        rep.check("TP stream alive during keyboard use", False,
                  f"only {len(tp_times)} TP reports at all")
        rep.data[res_prefix] = out
        return

    # gaps in the TP stream
    gaps = []  # (start, length)
    for a, b in zip(tp_times, tp_times[1:]):
        if b - a > 0.05:
            gaps.append((a, b - a))
    # does the stream die before the capture window ends?
    tail = cap.t_end - tp_times[-1]
    died = tail > 1.0
    if died:
        last_press_rel = (f"{presses[-1] - cap.t_start:.2f}s"
                          if presses else "n/a")
        rep.check("TP stream survives the whole capture", False,
                  f"stream DIED {tp_times[-1] - cap.t_start:.2f}s in "
                  f"(silent for the last {tail:.1f}s); last key press at "
                  f"{last_press_rel} — matches the missed-DR-edge stall "
                  f"(edge-triggered irq + wedged sysworkq)")
    else:
        rep.check("TP stream survives the whole capture", True,
                  f"reports until the end ({tail * 1e3:.0f}ms tail)")

    # correlation: gap starts near a key press (press within 150ms before
    # the gap or anywhere inside it)
    correlated = sum(1 for (g0, glen) in gaps
                     if any(g0 - 0.15 <= p <= g0 + glen for p in presses))
    frac = correlated / len(gaps) if gaps else 0.0
    rep.info(f"TP gaps >50ms: {len(gaps)}"
             + (f", {correlated} coincide with key presses ({frac:.0%})"
                if gaps else ""))
    if gaps:
        lens = sorted(g * 1e3 for _, g in gaps)
        rep.info(f"gap lengths ms: {stats_line(lens)} "
                 f"(HID_TX_TIMEOUT in usb_hid.c is 100ms)")
    rep.check("keyboard activity does not stall the touchpad",
              not died and (len(gaps) <= 2 or frac < 0.3),
              f"{len(gaps)} gaps, {frac:.0%} correlated with key presses",
              fail_detail="TP I2C reads starve while the keyboard submits "
              "block the shared sysworkq (hid_touchpad_gpio_cb -> "
              "k_work_submit; usb_hid.c HID_TX_TIMEOUT)")
    out.update({"gaps": len(gaps), "correlated": correlated, "died": died})
    rep.data[res_prefix] = out


def analyze_two_finger(cap, rep, res_prefix):
    ptp = [parse_ptp(p) for _t, rid, p in cap.hid_events if rid == PTP_ID]
    ptp = [f for f in ptp if f]
    max_tips = max((sum(fg["tip"] for fg in f["fingers"]) for f in ptp), default=0)
    max_cc = max((f["contact_count"] for f in ptp), default=0)
    cids = {fg["cid"] for f in ptp for fg in f["fingers"] if fg["tip"]}
    rep.check("two concurrent contacts reported", max_tips >= 2,
              f"max tips {max_tips}, max contact_count {max_cc}, "
              f"contact ids {sorted(cids)}")
    rep.data[res_prefix] = {"max_tips": max_tips, "max_contact_count": max_cc,
                            "contact_ids": sorted(cids)}


# ---------------------------------------------------------------- phases

def phase_enum(rep, want_bus, aster):
    rep.banner(f"1. enumeration ({'USB' if want_bus == BUS_USB else 'BLE'})")
    tp = None
    if want_bus == BUS_USB:
        usb = usb_sysfs_info()
        if not rep.check(f"USB device {VID:04x}:{PID:04x} present", usb is not None,
                         fail_detail="not enumerated — cable? known replug bug "
                         "(needs reset, see daisy.md)"):
            return None
        rep.info(f"bus {usb.get('busnum')} dev {usb.get('devnum')}, "
                 f"bcdDevice {usb.get('bcdDevice')} "
                 f"({usb.get('product', '?')})")
        rep.data["usb"] = {k: v for k, v in usb.items() if k != "syspath"}

    devs = find_daisy_hid(want_bus)
    for d in devs:
        rep.info(repr(d) + f"  name={d.name!r}")
    kinds = {d.kind: d for d in devs}
    rep.check("HID devices found", bool(devs), f"{len(devs)} hid device(s)")
    if want_bus == BUS_USB:
        rep.check("keyboard HID interface present", "keyboard" in kinds)
    if not rep.check("touchpad HID interface present (digitizer collection)",
                     "touchpad" in kinds,
                     fail_detail="firmware never registered the TP HID class — "
                     "hid_passthrough_usb_next.c / tp_hid node"):
        return None
    tp = kinds["touchpad"]

    # descriptor sanity
    rd = tp.rdesc
    ok_ids = (b"\x85\x01" in rd) and (b"\x85\x04" in rd)
    rep.check("report descriptor has mouse id 0x01 + PTP id 0x04", ok_ids,
              f"descriptor {len(rd)} bytes")
    rep.check("PTPHQA blob (feature 0x0A) in descriptor", b"\x85\x0a" in rd,
              warn_only=True)

    if want_bus == BUS_USB:
        rep.check("hid-multitouch bound", tp.driver == "hid-multitouch",
                  f"driver={tp.driver}",
                  fail_detail="without it there is no PTP evdev; compare "
                  "~/clone/linux/drivers/hid/hid-multitouch.c probe")
    else:
        rep.check("kernel driver bound", tp.driver is not None,
                  f"driver={tp.driver}")
    rep.check("evdev input node(s) created", bool(tp.event_nodes),
              ", ".join(f"{n} ({m})" for n, m in tp.event_nodes))
    rep.check("hidraw node present", tp.hidraw is not None, str(tp.hidraw))

    if aster and want_bus == BUS_USB:
        try:
            out = subprocess.run([aster, "v1", "--endpoint-get"],
                                 capture_output=True, text=True, timeout=10)
            ep = (out.stdout + out.stderr).strip().replace("\n", " | ")
            rep.info(f"endpoint (aster): {ep}")
            rep.data["endpoint"] = ep
            if "ble" in ep.lower() and "usb" not in ep.lower().split("selected")[-1]:
                rep.info(C.c(C.Y, "NOTE: reports may be routed to BLE right now — "
                                  "USB input phase would see nothing!"))
        except (OSError, subprocess.TimeoutExpired) as e:
            rep.info(f"(aster endpoint check unavailable: {e})")

    kernel_log_hints(rep)
    return tp


def kernel_log_hints(rep):
    for cmd in (["dmesg", "--since", "-5min"],
                ["journalctl", "-k", "--since", "-5 min", "--no-pager", "-q"]):
        try:
            out = subprocess.run(cmd, capture_output=True, text=True, timeout=5)
            if out.returncode != 0:
                continue
            bad = [l for l in out.stdout.splitlines()
                   if re.search(r"32ac|multitouch|hid", l, re.I)
                   and re.search(r"error|fail|timed? ?out|invalid|refus", l, re.I)]
            for l in bad[-8:]:
                rep.info(f"kernel: {l.strip()}")
            return
        except OSError:
            continue


def phase_features(rep, tp, args, tag):
    rep.banner(f"2. feature reports over hidraw ({tag})")
    if tp.hidraw is None:
        rep.skip("all feature checks", "no hidraw node")
        return
    try:
        fd = os.open(tp.hidraw, os.O_RDWR)
    except OSError as e:
        rep.check("open hidraw read/write", False, f"{tp.hidraw}: {e.strerror}")
        return
    try:
        results = {}
        for rid, (label, size, required) in FEATURES.items():
            try:
                payload, dt = hidraw_get_feature(fd, rid, size)
                results[rid] = {"ok": True, "ms": dt * 1e3, "len": len(payload),
                                "head": payload[:8].hex()}
                rep.check(f"GET_FEATURE 0x{rid:02x} ({label})", True,
                          f"{len(payload)}B in {fmt_ms(dt)}, data {payload[:8].hex()}"
                          + ("..." if len(payload) > 8 else ""))
            except OSError as e:
                results[rid] = {"ok": False, "err": e.strerror}
                rep.check(f"GET_FEATURE 0x{rid:02x} ({label})", False,
                          f"{e.strerror} — get_report_cb -> hid_touchpad_get_report "
                          f"(I2C) failing?", warn_only=not required)
        rep.data[f"features_{tag}"] = results

        # cross-check payload sizes against the descriptor (catches e.g. the
        # BLE off-by-one where every feature read loses its last byte)
        short = [rid for rid, (_l, size, _req) in FEATURES.items()
                 if results.get(rid, {}).get("ok")
                 and results[rid]["len"] < size]
        # A systematic 1-byte-short pattern across ALL reports = the BlueZ
        # bt_uhid_get_report_reply off-by-one; isolated short reads on
        # optional reports are just pad quirks (0x07 returns empty on USB).
        systematic = len(short) >= len(FEATURES) - 2
        rep.check("feature payloads have full descriptor-declared size",
                  not short,
                  "short: " + ", ".join(
                      f"0x{r:02x} ({results[r]['len']}/{FEATURES[r][1]}B)"
                      for r in short) if short else "all match",
                  warn_only=not systematic,
                  fail_detail="all reports short by one = BlueZ hog "
                  "get-report off-by-one (src/shared/uhid.c)"
                  if systematic else None)

        # interpret the two most meaningful ones
        if results.get(0x02, {}).get("ok") and results[0x02]["head"]:
            cmax = int(results[0x02]["head"][:2], 16)
            rep.check("contact max sane (2..5)", 2 <= cmax <= 5, f"{cmax}")
        mode = None
        if results.get(0x03, {}).get("ok") and results[0x03]["head"]:
            mode = int(results[0x03]["head"][:2], 16)
            rep.check(f"input mode == {INPUT_MODE_PTP} (PTP)", mode == INPUT_MODE_PTP,
                      f"mode={mode}", warn_only=True,
                      fail_detail="hid-multitouch sets this at probe; if it is 0 "
                      "the pad only sends mouse reports (set_report path broken?)")

        # latency statistics on a small round-trip
        reps = args.feature_reps
        lat = []
        for _ in range(reps):
            try:
                _, dt = hidraw_get_feature(fd, 0x03, 1)
                lat.append(dt * 1e3)
            except OSError:
                break
        if lat:
            rep.info(f"GET_FEATURE(input mode) ×{len(lat)}: {stats_line(lat)}")
            rep.data[f"feature_latency_ms_{tag}"] = {
                "n": len(lat), "min": min(lat),
                "med": statistics.median(lat), "max": max(lat)}

        # set_report path: write input mode PTP and read back (idempotent)
        if args.set_test and mode is not None:
            try:
                dt = hidraw_set_feature(fd, 0x03, [INPUT_MODE_PTP])
                back, _ = hidraw_get_feature(fd, 0x03, 1)
                rep.check("SET_FEATURE input mode -> readback",
                          back and back[0] == INPUT_MODE_PTP,
                          f"wrote {INPUT_MODE_PTP} in {fmt_ms(dt)}, read back "
                          f"{back.hex() if back else '?'}")
            except OSError as e:
                rep.check("SET_FEATURE input mode -> readback", False,
                          f"{e.strerror} — set_report_cb -> I2C path")
        elif args.set_test:
            rep.skip("SET_FEATURE input mode", "GET input mode failed")
    finally:
        os.close(fd)


def phase_input(rep, tp, args, tag, record_fh):
    rep.banner(f"3. live input ({tag})")
    if tp.hidraw is None:
        rep.skip("input capture", "no hidraw node")
        return
    if not os.access(tp.hidraw, os.R_OK):
        rep.check("open hidraw for reading", False, "permission denied")
        return

    if ask(f"MOTION: move ONE finger in circles over the WHOLE pad for "
           f"{args.duration}s, without lifting"):
        with Capture(tp, record_fh, tag) as cap:
            cap.run(args.duration)
        analyze_motion(cap, rep, f"motion_{tag}")
    else:
        rep.skip("motion capture", "user skipped")

    if args.quick_input:
        return
    if ask("TAP: single quick tap in the middle of the pad (3s window)"):
        with Capture(tp, record_fh, tag) as cap:
            cap.run(3)
        analyze_tap(cap, rep, f"tap_{tag}")
    else:
        rep.skip("tap capture", "user skipped")

    if ask("TWO-FINGER: rest/move two fingers on the pad (4s window)"):
        with Capture(tp, record_fh, tag) as cap:
            cap.run(4)
        analyze_two_finger(cap, rep, f"two_finger_{tag}")
    else:
        rep.skip("two-finger capture", "user skipped")

    # keyboard interference: TP I2C reads share the sysworkq with the
    # keyboard HID submit path, so key presses can starve/kill the stream
    bus = tp.bus
    kb_nodes = [n for d in find_daisy_hid(bus) if d.kind == "keyboard"
                for n, _name in d.event_nodes]
    if not kb_nodes:
        rep.skip("keyboard interference capture", "no daisy keyboard evdev node")
    elif ask(f"INTERFERENCE: keep circling ONE finger on the pad AND tap "
             f"letter keys on the DAISY keyboard ~2x per second, both for "
             f"{args.duration}s"):
        with Capture(tp, record_fh, tag,
                     extra_nodes=[(n, "kb") for n in kb_nodes]) as cap:
            cap.run(args.duration)
        analyze_interference(cap, rep, f"interference_{tag}", kb_nodes)
    else:
        rep.skip("keyboard interference capture", "user skipped")


def watch(tp):
    print(f"watching {tp.hidraw} ({tp.name}) — ^C to stop", flush=True)
    fd = os.open(tp.hidraw, os.O_RDONLY)
    last_t, last_scan = None, None
    try:
        while True:
            data = os.read(fd, 4096)
            t = time.monotonic()
            dt = f"{(t - last_t) * 1e3:7.2f}ms" if last_t else "        "
            last_t = t
            rid, p = data[0], data[1:]
            if rid == PTP_ID and (f := parse_ptp(p)):
                sdt = ""
                if last_scan is not None:
                    d = (f["scan_time"] - last_scan) & 0xFFFF
                    sdt = f" scan+{d * SCAN_TIME_UNIT_S * 1e3:6.2f}ms"
                last_scan = f["scan_time"]
                tips = [(fg["cid"], fg["x"], fg["y"])
                        for fg in f["fingers"] if fg["tip"]]
                print(f"{dt}{sdt} PTP cc={f['contact_count']} btn={f['buttons']} "
                      + " ".join(f"[{c}]{x},{y}" for c, x, y in tips))
            elif rid == MOUSE_ID and (m := parse_mouse(p)):
                print(f"{dt}          MOUSE btn={m['buttons']} dx={m['dx']} "
                      f"dy={m['dy']} wheel={m['wheel']} pan={m['pan']}")
            else:
                print(f"{dt} id=0x{rid:02x} {p.hex()}")
    except KeyboardInterrupt:
        print()
    finally:
        os.close(fd)


# ---------------------------------------------------------------- gain

# " event24  POINTER_MOTION  [count]  +1.03s  -4.88/ -1.95 ( -5.00/ -2.00)"
# The count column appears from the 2nd event on; values may carry a '+'.
LIBINPUT_MOTION_RE = re.compile(
    r"POINTER_MOTION\s+(?:\d+\s+)?\+?[\d.]+s\s+([+-]?[\d.]+)\s*/\s*([+-]?[\d.]+)"
    r"\s+\(\s*([+-]?[\d.]+)\s*/\s*([+-]?[\d.]+)")


def device_travel_mm(hid_events, res=12):
    """Total finger travel in mm from consecutive PTP frames (first tip)."""
    total, prev = 0.0, None
    for _t, rid, p in hid_events:
        if rid != PTP_ID:
            continue
        f = parse_ptp(p)
        if not f:
            continue
        tips = [fg for fg in f["fingers"] if fg["tip"]]
        cur = (tips[0]["x"], tips[0]["y"]) if tips else None
        if prev and cur:
            total += ((cur[0] - prev[0]) ** 2 + (cur[1] - prev[1]) ** 2) ** 0.5
        prev = cur
    return total / res


def parse_libinput_lines(t, lines):
    for raw in lines:
        if t.get("rawlog"):
            t["rawlog"].write(raw + b"\n")
        line = raw.decode(errors="replace")
        parts = line.split()
        # "event24  POINTER_MOTION  +1.03s  0.51/0.00 (...)" or "-event24 ..."
        etype = None
        for tok in parts[:3]:
            if tok.isupper() and "_" in tok:
                etype = tok
                break
        if etype:
            t["event_types"][etype] = t["event_types"].get(etype, 0) + 1
            if len(t["samples"]) < 4 and etype not in (
                    s.split()[1] if len(s.split()) > 1 else "" for s in t["samples"]):
                t["samples"].append(line.rstrip()[:110])
        m = LIBINPUT_MOTION_RE.search(line)
        if m:
            ax, ay, ux, uy = map(float, m.groups())
            t["accel"] += (ax * ax + ay * ay) ** 0.5
            t["unaccel"] += (ux * ux + uy * uy) ** 0.5
            t["motion_lines"] += 1
        elif etype == "POINTER_MOTION" and len(t.setdefault("unmatched", [])) < 3:
            t["unmatched"].append(repr(raw))


def gain_mode(args):
    """Measure libinput pointer output per mm of finger travel, per transport.

    With USB plugged and BLE connected the firmware feeds both transports at
    once, so a single swipe session yields a controlled A/B comparison."""
    import shutil
    if not shutil.which("libinput"):
        sys.exit("libinput binary not found")
    targets = []
    for bus, tag in ((BUS_USB, "usb"), (BUS_BT, "ble")):
        for d in find_daisy_hid(bus):
            if d.kind != "touchpad":
                continue
            nodes = [n for n, name in d.event_nodes if name.endswith("Touchpad")]
            if nodes and d.hidraw:
                targets.append({"tag": tag, "dev": d, "node": nodes[0]})
    if not targets:
        sys.exit("no daisy touchpad devices found")
    if os.geteuid() != 0 and not args.no_sudo and sys.stdin.isatty():
        os.execvp("sudo", ["sudo", sys.executable] + sys.argv)

    print("gain measurement targets:")
    for t in targets:
        print(f"  {t['tag']}: hidraw={t['dev'].hidraw} evdev={t['node']}")

    procs = []
    for t in targets:
        p = subprocess.Popen(
            ["libinput", "debug-events", "--device", t["node"],
             f"--set-profile={args.gain_profile}"],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
        os.set_blocking(p.stdout.fileno(), False)
        procs.append(p)
        t["proc"] = p
        t["buf"] = b""
        t["accel"] = t["unaccel"] = 0.0
        t["motion_lines"] = 0
        t["event_types"] = {}
        t["samples"] = []
        logpath = f"/tmp/libinput_gain_{t['tag']}.log"
        t["rawlog"] = open(logpath, "wb")
        t["rawlog_path"] = logpath
        t["fd"] = os.open(t["dev"].hidraw, os.O_RDONLY | os.O_NONBLOCK)
        t["hid_events"] = []
    time.sleep(0.5)  # let libinput contexts settle

    print(f"\n  >>> Swipe ONE finger around the pad continuously for "
          f"{args.duration:.0f}s (big slow circles)")
    try:
        input("      Press Enter to start... ")
    except EOFError:
        pass
    end = time.monotonic() + args.duration
    fd_map = {t["fd"]: t for t in targets}
    out_map = {t["proc"].stdout.fileno(): t for t in targets}
    try:
        while time.monotonic() < end:
            r, _, _ = select.select(list(fd_map) + list(out_map), [], [], 0.05)
            for fd in r:
                if fd in fd_map:
                    t = fd_map[fd]
                    try:
                        data = os.read(fd, 4096)
                        if data:
                            t["hid_events"].append(
                                (time.monotonic(), data[0], data[1:]))
                    except OSError:
                        pass
                else:
                    t = out_map[fd]
                    try:
                        chunk = os.read(fd, 65536)
                    except OSError:
                        continue
                    t["buf"] += chunk
                    *lines, t["buf"] = t["buf"].split(b"\n")
                    parse_libinput_lines(t, lines)
    finally:
        for t in targets:
            t["proc"].terminate()
            os.close(t["fd"])
        time.sleep(0.3)
        for t in targets:  # scoop anything still buffered in the pipe
            try:
                rest = t["proc"].stdout.read()
                if rest:
                    lines = (t["buf"] + rest).split(b"\n")
                    parse_libinput_lines(t, lines)
            except (OSError, TypeError, ValueError):
                pass

    for t in targets:
        t["rawlog"].close()
        types = ", ".join(f"{k}×{v}" for k, v in
                          sorted(t["event_types"].items(), key=lambda kv: -kv[1]))
        print(f"\n  {t['tag']} libinput events: {types or 'NONE'} "
              f"(raw log: {t['rawlog_path']})")
        for s in t["samples"]:
            print(f"    {s}")
        for u in t.get("unmatched", []):
            print(f"    UNMATCHED motion line: {u}")

    print(f"\n  {'':6} {'frames':>7} {'travel':>9} {'events':>7} "
          f"{'unaccel':>9} {'accel':>9} {'unacc/mm':>9} {'acc/mm':>8}")
    results = {}
    for t in targets:
        mm = device_travel_mm(t["hid_events"])
        ua = t["unaccel"] / mm if mm else 0
        ac = t["accel"] / mm if mm else 0
        results[t["tag"]] = (mm, ua, ac)
        print(f"  {t['tag']:6} {len(t['hid_events']):7} {mm:8.1f}mm "
              f"{t['motion_lines']:7} {t['unaccel']:9.1f} {t['accel']:9.1f} "
              f"{ua:9.3f} {ac:8.3f}")
    if "usb" in results and "ble" in results and results["usb"][1]:
        u, b = results["usb"], results["ble"]
        print(f"\n  BLE/USB ratio: unaccelerated {b[1]/u[1]:.2f}x, "
              f"accelerated {b[2]/u[2]:.2f}x  ({args.gain_profile} profile)")
        print("  <1.0 means the BLE pointer genuinely travels less per mm "
              "of finger motion at the libinput level.")
    return 0


# ---------------------------------------------------------------- diagnosis

def diagnose(rep):
    rep.banner("summary")
    fails = [c for c in rep.checks if c[2] == "fail"]
    warns = [c for c in rep.checks if c[2] == "warn"]
    npass = sum(1 for c in rep.checks if c[2] == "pass")
    print(f"  {npass} passed, {len(fails)} failed, {len(warns)} warnings")

    def failed(sub):
        return any(sub in name for _, name, st, _ in rep.checks if st == "fail")

    verdict = []
    if failed("USB device 32ac:0034"):
        verdict.append("Device not on the bus at all: cable/power, or the known "
                       "no-re-enumeration-after-replug bug (reset via "
                       "`nrfutil device reset`). Firmware: app/src/usb.c VBUS handling.")
    elif failed("touchpad HID interface"):
        verdict.append("TP HID class never enumerated: firmware registration. Check "
                       "RTT logs for 'hid_device_register failed' / 'TP HID device "
                       "not ready' (hid_passthrough_usb_next.c, tp_hid DT node).")
    elif failed("GET_FEATURE 0x02") or failed("GET_FEATURE 0x0a"):
        verdict.append("Control path to the physical touchpad is broken: USB reaches "
                       "the firmware but get_report_cb -> hid_touchpad_get_report "
                       "(I2C) fails. Check RTT for 'get_report proxy failed' and "
                       "-ENOTSUP ('Touchpad device not ready' at boot = I2C probe "
                       "failed). Firmware: hid_touchpad.c / I2C bus / power to pad.")
    elif failed("raw input reports arrive"):
        verdict.append("Control path OK but no input reports: interrupt path. "
                       "DR gpio not firing, hid_touchpad input cb not called, "
                       "hid_ready false (iface_ready never fired), TX semaphore "
                       "starved ('TX buffer still in flight' in RTT), or reports "
                       "routed to the other endpoint (aster --endpoint-get). "
                       "Firmware: hid_touchpad.c ISR + hid_passthrough_*.c.")
    elif failed("PTP (report id 0x04)"):
        verdict.append("Pad is stuck in mouse mode: input mode feature (id 3) not "
                       "set/persisted — set_report path or pad reset after mode set. "
                       "See SET_FEATURE check; strace hid-multitouch probe or "
                       "re-plug while watching RTT set_report logs.")
    elif failed("does not stall the touchpad") or failed("survives the whole"):
        verdict.append("Keyboard activity starves/kills the TP stream: the TP "
                       "I2C read work runs on the system workqueue "
                       "(hid_touchpad.c: gpio cb -> k_work_submit) while the "
                       "keyboard HID submit can block that queue up to "
                       "HID_TX_TIMEOUT=100ms (usb_hid.c). If the stream dies "
                       "outright: DR is edge-triggered (EDGE_TO_ACTIVE) — a "
                       "report arriving while the queue is wedged leaves the "
                       "line asserted, no new edge, never read again. "
                       "Candidate fixes: dedicated TP workqueue (like "
                       "daisy_pairing_led), and drain-until-DR-deasserts / "
                       "level-triggered irq.")
    elif failed("kernel driver emits evdev"):
        verdict.append("Raw PTP frames are fine but hid-multitouch produces no "
                       "events: host-side parsing. Capture with hid-recorder "
                       "(~/clone/hid-tools) and replay/compare; check "
                       "hid-multitouch quirks for VID 32ac.")
    elif failed("hid-multitouch bound"):
        verdict.append("hid-generic grabbed the TP interface: descriptor doesn't "
                       "look like a win8 PTP to the kernel, or hid-multitouch not "
                       "loaded (modprobe hid-multitouch).")
    elif not fails:
        verdict.append("All layers pass — communication looks healthy. If gestures "
                       "still misbehave, next layer up is libinput: "
                       "`sudo libinput debug-events --device /dev/input/eventX` "
                       "and `libinput record` (~/clone/libinput).")
    else:
        verdict.append("See failed checks above.")

    for v in verdict:
        print(f"\n  {C.c(C.Y, 'VERDICT:')} {v}")
    rep.data["verdict"] = verdict
    rep.data["counts"] = {"pass": npass, "fail": len(fails), "warn": len(warns)}
    return 0 if not fails else 1


# ---------------------------------------------------------------- main

def maybe_sudo(args, tp):
    if os.geteuid() == 0 or args.no_sudo:
        return
    needs = [p for p in ([tp.hidraw] + [n for n, _ in tp.event_nodes]) if p]
    if all(os.access(p, os.R_OK) for p in needs):
        return
    if not sys.stdin.isatty():
        print(C.c(C.Y, "hidraw/evdev not readable and no tty for sudo — "
                       "run as root for phases 2+"))
        return
    print(C.c(C.Y, "re-executing under sudo for hidraw/evdev access "
                   "(--no-sudo to disable)..."))
    os.execvp("sudo", ["sudo", sys.executable] + sys.argv)


def run_transport(rep, args, bus, tag, record_fh):
    tp = phase_enum(rep, bus, args.aster)
    if tp is None:
        return None
    if bus == BUS_USB:
        maybe_sudo(args, tp)
    phase_features(rep, tp, args, tag)
    if not args.quick:
        phase_input(rep, tp, args, tag, record_fh)
    return tp


def lag_probe(args):
    """Snapshot the touchpad-lag state. Run this WHILE the pad feels laggy,
    BEFORE opening tapview/evtest/anything else.

    Probe order matters: sysfs and /proc reads don't touch the device, but
    opening hidraw (done last) triggers hid_hw_open -> USB runtime resume ->
    hid-multitouch rewriting the mode features — the same mechanism that makes
    "open tapview" un-stick the pad — so it can destroy the state being
    observed. If the pad becomes snappy the moment this script reaches step 3,
    that is itself the answer.
    """
    devs = [d for d in find_daisy_hid(BUS_USB) if d.kind == "touchpad"]
    if not devs:
        sys.exit("no touchpad HID device on USB (dongle plugged in? --dongle?)")
    tp = devs[0]
    print(f"touchpad: {tp.hid_id} hidraw={tp.hidraw} "
          f"events={[p for p, _ in tp.event_nodes]}")

    print("\n-- 1. USB runtime PM (sysfs, non-invasive) --")
    usb = usb_sysfs_info()
    if usb:
        for attr in ("power/control", "power/runtime_status",
                     "power/autosuspend_delay_ms", "power/wakeup"):
            try:
                with open(os.path.join(usb["syspath"], attr)) as f:
                    print(f"  {attr}: {f.read().strip()}")
            except OSError as e:
                print(f"  {attr}: unreadable ({e})")
    else:
        print("  USB device not found in sysfs?!")

    print("\n-- 2. processes holding the input nodes (non-invasive) --")
    targets = {p for p, _ in tp.event_nodes}
    if tp.hidraw:
        targets.add(tp.hidraw)
    holders = {t: [] for t in targets}
    for pid in filter(str.isdigit, os.listdir("/proc")):
        try:
            comm = open(f"/proc/{pid}/comm").read().strip()
            for fd in os.listdir(f"/proc/{pid}/fd"):
                try:
                    tgt = os.readlink(f"/proc/{pid}/fd/{fd}")
                except OSError:
                    continue
                if tgt in holders:
                    holders[tgt].append(f"{comm}({pid})")
        except OSError:
            continue
    for t in sorted(holders):
        who = ", ".join(holders[t]) if holders[t] else "NOBODY"
        print(f"  {t}: {who}")
    tp_ev = [p for p, n in tp.event_nodes if "Touchpad" in n]
    if tp_ev and not holders.get(tp_ev[0]):
        print(C.c(C.Y, "  !! nothing holds the Touchpad event node — libinput "
                       "hasn't claimed it; hid-multitouch may be idle/suspended"))

    print("\n-- 3. hidraw sample, 3 s (opens hidraw — may un-stick the pad!) --")
    print("   keep a finger moving on the pad...")
    counts, scan, arrivals = {}, [], []
    t0 = time.monotonic()
    with open(tp.hidraw, "rb", buffering=0) as f:
        os.set_blocking(f.fileno(), False)
        while time.monotonic() - t0 < 3.0:
            r, _, _ = select.select([f], [], [], 0.2)
            if not r:
                continue
            buf = f.read(64)
            if not buf:
                continue
            counts[buf[0]] = counts.get(buf[0], 0) + 1
            if buf[0] == PTP_ID and len(buf) >= 1 + PTP_PAYLOAD:
                p = parse_ptp(buf[1:])
                if p:
                    scan.append(p["scan_time"])
                    arrivals.append(time.monotonic())
    dur = time.monotonic() - t0
    for rid, n in sorted(counts.items()):
        print(f"  report id 0x{rid:02x}: {n} frames, {n / dur:.1f} Hz")
    if len(scan) > 2:
        deltas = [(b - a) & 0xFFFF for a, b in zip(scan, scan[1:])]
        med = statistics.median(deltas) * SCAN_TIME_UNIT_S * 1000
        print(f"  PTP scan-time median delta: {med:.1f} ms "
              f"({1000 / med:.0f} Hz at the pad itself)")
    if len(arrivals) > 2:
        # Arrival shape: uniform-slow vs bursty tells apart a steady drain
        # bottleneck (median ~= p95) from link starvation delivering queued
        # frames in clumps (tiny median, huge p95).
        adel = sorted((b - a) * 1000 for a, b in zip(arrivals, arrivals[1:]))
        med = statistics.median(adel)
        p95 = adel[int(len(adel) * 0.95)]
        burst = sum(1 for d in adel if d < med / 2)
        print(f"  arrival deltas: median {med:.1f} ms, p95 {p95:.1f} ms, "
              f"max {adel[-1]:.1f} ms")
        if p95 > 4 * med:
            print(C.c(C.Y, f"  !! BURSTY delivery ({burst} tight pairs): frames "
                           "queue upstream and arrive in clumps -> BLE link "
                           "starvation (kb-side TX or radio contention)"))
        else:
            print("  delivery is uniform -> steady drain bottleneck, not bursts")
    if not counts:
        print("  no frames at all — pad silent or interface suspended")

    print("\n-- verdict hints --")
    print("  id 0x04 @ ~130 Hz  -> healthy (or step 3 just un-stuck it)")
    print("  id 0x04 @ ~20 Hz   -> pad in high-latency scan (suspend/latency chain)")
    print("  id 0x01 (mouse)    -> pad fell back to mouse mode (input-mode lost)")
    print("  runtime_status=suspended + NOBODY on event node -> autosuspend chain")
    print("  now open tapview while watching the dongle serial log for a")
    print("  usbd_set_report_cb id=3/5/7 burst (= host rewriting modes)")
    return 0


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ble", action="store_true",
                    help="also test the Bluetooth transport (device must be "
                    "bonded+connected; use bond_dongle.py --laptop --prep)")
    ap.add_argument("--ble-only", action="store_true",
                    help="skip USB phases, test only Bluetooth")
    ap.add_argument("--dongle", action="store_true",
                    help="target the USB-A Bluetooth dongle's touchpad "
                    "passthrough interface (32ac:0039) instead of the "
                    "keyboard's own USB. The BLE leg terminates at the "
                    "dongle, so this is USB-only.")
    ap.add_argument("--quick", action="store_true",
                    help="enumeration + feature reports only (no interaction)")
    ap.add_argument("--quick-input", action="store_true",
                    help="motion capture only, skip tap/two-finger prompts")
    ap.add_argument("--duration", type=float, default=8,
                    help="seconds of motion capture (default 8)")
    ap.add_argument("--feature-reps", type=int, default=10,
                    help="repetitions for feature-report latency stats")
    ap.add_argument("--no-set-test", dest="set_test", action="store_false",
                    help="skip the SET_FEATURE(input mode) test")
    ap.add_argument("--record", metavar="FILE",
                    help="append raw hidraw captures as JSONL")
    ap.add_argument("--json", metavar="FILE", help="write result summary as JSON")
    ap.add_argument("--gain", action="store_true",
                    help="measure libinput pointer output per mm of finger "
                    "travel on USB and BLE simultaneously")
    ap.add_argument("--gain-profile", choices=("flat", "adaptive"),
                    default="flat",
                    help="accel profile for --gain (flat isolates delta "
                    "processing; adaptive includes velocity estimation + "
                    "accel curve, i.e. the real desktop feel)")
    ap.add_argument("--lag-probe", action="store_true",
                    help="snapshot the lag state (run WHILE laggy, before "
                    "opening tapview): USB runtime PM, input-node holders, "
                    "then a 3s hidraw sample — in that order, least-invasive "
                    "first")
    ap.add_argument("--watch", action="store_true",
                    help="live parsed dump of the touchpad hidraw stream")
    ap.add_argument("--watch-ble", action="store_true",
                    help="like --watch but on the Bluetooth hid device")
    ap.add_argument("--no-sudo", action="store_true",
                    help="never re-exec with sudo")
    ap.add_argument("--aster", default=os.path.expanduser(
        "~/clone/aster/target/debug/aster"),
        help="path to aster for endpoint info (optional)")
    args = ap.parse_args()
    if not os.path.exists(args.aster):
        args.aster = None

    if args.dongle:
        global PID
        PID = 0x0039  # Framework USB-A Bluetooth Dongle
        if args.ble or args.ble_only or args.watch_ble:
            sys.exit("--dongle is USB-only: the BLE leg terminates at the "
                     "dongle, the host only sees its USB interfaces")
        args.aster = None  # aster talks to the keyboard, not the dongle

    if args.lag_probe:
        if os.geteuid() != 0 and not args.no_sudo and sys.stdin.isatty():
            os.execvp("sudo", ["sudo", sys.executable] + sys.argv)
        return lag_probe(args)

    if args.gain:
        return gain_mode(args)

    if args.watch or args.watch_ble:
        bus = BUS_BT if args.watch_ble else BUS_USB
        devs = [d for d in find_daisy_hid(bus) if d.kind == "touchpad"]
        if not devs:
            sys.exit(f"no daisy touchpad hid device on "
                     f"{'BT' if bus == BUS_BT else 'USB'}")
        if not os.access(devs[0].hidraw, os.R_OK) and os.geteuid() != 0 \
                and not args.no_sudo and sys.stdin.isatty():
            os.execvp("sudo", ["sudo", sys.executable] + sys.argv)
        watch(devs[0])
        return 0

    rep = Report()
    record_fh = open(args.record, "a") if args.record else None
    print(f"Daisy touchpad diagnostic — {time.strftime('%Y-%m-%d %H:%M:%S')}")

    try:
        if not args.ble_only:
            run_transport(rep, args, BUS_USB, "usb", record_fh)
        if args.ble or args.ble_only:
            note = (">>> BLE transport: device must be bonded + connected to "
                    "this host, and input routed to BLE (aster v1 --endpoint "
                    "ble). See bond_dongle.py --laptop.")
            print("\n" + C.c(C.Y, note))
            if not args.quick:
                try:
                    input("    Press Enter to continue with BLE... ")
                except EOFError:
                    pass
            run_transport(rep, args, BUS_BT, "ble", record_fh)
            # cross-transport comparison
            u = rep.data.get("feature_latency_ms_usb")
            b = rep.data.get("feature_latency_ms_ble")
            if u and b:
                rep.banner("USB vs BLE")
                rep.info(f"feature round-trip med: USB {u['med']:.2f}ms, "
                         f"BLE {b['med']:.2f}ms")
            mu = rep.data.get("motion_usb", {}).get("rate_hz")
            mb = rep.data.get("motion_ble", {}).get("rate_hz")
            if mu and mb:
                rep.info(f"motion report rate: USB {mu} Hz, BLE {mb} Hz")
    finally:
        if record_fh:
            record_fh.close()

    code = diagnose(rep)
    if args.json:
        with open(args.json, "w") as f:
            json.dump({"checks": [
                {"phase": p, "name": n, "status": s, "detail": d}
                for p, n, s, d in rep.checks], **rep.data}, f, indent=2)
        print(f"\n  JSON written to {args.json}")
    return code


if __name__ == "__main__":
    sys.exit(main())
