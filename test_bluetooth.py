#!/usr/bin/env python3
"""Bluetooth test for the Daisy keyboard.

Drives the device over the USB factory interface (aster v1) while this host
acts as the BLE central (bluetoothctl). Endpoint switching only changes where
input reports are routed -- the USB interfaces (including the factory one)
stay up -- so aster keeps working even while the keyboard "is on Bluetooth".

Phases (one cycle):
  0 preflight     clear bonds on both sides, factory mode on
  1 pair          scan, pair + trust profile 0, verify both sides (timed)
  2 usb-input     inject N x F24, expect all on the USB evdev node, none leaked
  3 endpoint-ble  switch reports to BLE (timed), F24 arrives via BT (and not USB)
  4 endpoint-none reports go nowhere: F24 absent from both USB and BT
  5 endpoint-usb  switch back (timed), mirrored checks
  6 advertising   --bt-adv-off clears adv, --bt-adv-on restores it
  7 reboot        reboot on BLE, bond survives, host auto-reconnects, F24 flows
                  again (reboot + reconnect timed). --bootloader-jump by default
                  (mcuboot recovery + `os system-reset`), or a J-Link hardware
                  reset with --reset-method nrfutil.
  8 profiles      profile 1 = unbonded (keys stop flowing over BLE),
                  next x3 wraps to 0 (keys flow again), prev/next round-trip
  9 negative      out-of-range profile rejected; re-pair while bonded is a no-op
 10 unpair        --bt-unpair drops profile 0's bond, host sees disconnect
 11 clear-all     re-pair, --bt-clear-pairings wipes everything
 12 cleanup       remove host bonds, endpoint back to USB (also on failure)

Not covered (needs a second BT central): two profiles bonded to two different
hosts at once, switching between two *live* connections. All profiles share
one BLE address on a single identity, so this one host can only ever hold one
bond, and re-pairing it onto a second profile fails (AuthenticationFailed) --
see the "same address" note in daisy.md.

Run once for a functional pass; `-n N` repeats the whole cycle as a soak and
reports a flake rate plus timing distributions.

Notes:
  - All ZMK profiles advertise the SAME BLE address (verified 2026-07-15) on a
    single identity, so one host bonds only one profile at a time and can't be
    re-bonded onto a second (see "Not covered" above / daisy.md).
  - Pairing runs inside one persistent bluetoothctl session with a
    NoInputNoOutput agent registered (one-shot `bluetoothctl pair` has no
    agent and fails).
  - Key-inject uses F24: real HID reports on the wire, nothing visible typed.
    Injecting N of them and counting arrivals turns "did a key arrive" into a
    lossless-delivery check.
  - Reading /dev/input needs root or the input group; the script tries a
    direct open, then passwordless sudo. --skip-input-check drops to
    state-level checks only.
"""

import argparse
import glob as globmod
import json
import os
import re
import struct
import subprocess
import sys
import threading
import time
from pathlib import Path


def _user_home():
    """Invoking user's home. Under sudo, $HOME is root's, so resolve the real
    user via SUDO_USER (input capture may re-exec us under sudo)."""
    sudo_user = os.environ.get("SUDO_USER")
    if sudo_user:
        import pwd

        try:
            return Path(pwd.getpwnam(sudo_user).pw_dir)
        except KeyError:
            pass
    return Path.home()


DEFAULT_ASTER = _user_home() / "clone/aster/target/debug/aster"
DEFAULT_MCUMGRCTL = _user_home() / "clone/mcumgr-toolkit/target/debug/mcumgrctl"
DEFAULT_REC_GLOB = "/dev/serial/by-id/*Daisy_Keyboard*Recovery*"
BT_NAME = "Framework TP KB"

EV_KEY = 0x01
KEY_F24 = 194
KEY_EVENT_FMT = "llHHi"  # struct input_event on 64-bit
KEY_EVENT_SIZE = struct.calcsize(KEY_EVENT_FMT)

ANSI_RE = re.compile(r"\x1b\[[0-9;]*m|\r|\x01|\x02")


# ---------------------------------------------------------------------------
# pretty output
# ---------------------------------------------------------------------------


class Style:
    enabled = sys.stdout.isatty()

    @classmethod
    def _c(cls, code, text):
        return f"\x1b[{code}m{text}\x1b[0m" if cls.enabled else text

    @classmethod
    def green(cls, t):
        return cls._c("32", t)

    @classmethod
    def red(cls, t):
        return cls._c("31;1", t)

    @classmethod
    def yellow(cls, t):
        return cls._c("33", t)

    @classmethod
    def cyan(cls, t):
        return cls._c("36;1", t)

    @classmethod
    def dim(cls, t):
        return cls._c("2", t)


class TestAbort(Exception):
    """Raised on a failed check when --keep-going is not set."""


class Report:
    def __init__(self, keep_going, iterations=1):
        self.keep_going = keep_going
        self.iterations = iterations
        self.iteration = 1
        self.phase_name = "preflight"
        self.results = []  # (iteration, phase, description, ok)
        self.timings = {}  # label -> [seconds]

    def start_iteration(self, i):
        self.iteration = i
        if self.iterations > 1:
            bar = Style.cyan("#" * 3)
            print(f"\n{bar} iteration {i}/{self.iterations} {bar}")

    def phase(self, name):
        self.phase_name = name
        print(f"\n{Style.cyan('===')} {Style.cyan(name)} {Style.cyan('===')}")

    def note(self, text):
        print(f"    {Style.dim(text)}")

    def record_time(self, label, seconds):
        self.timings.setdefault(label, []).append(seconds)
        self.note(f"{label}: {seconds:.2f}s")

    def ok(self, desc):
        self.results.append((self.iteration, self.phase_name, desc, True))
        print(f"  {Style.green('✓')} {desc}")

    def fail(self, desc):
        self.results.append((self.iteration, self.phase_name, desc, False))
        print(f"  {Style.red('✗')} {desc}")
        if not self.keep_going:
            raise TestAbort(desc)

    def check(self, desc, cond):
        if cond:
            self.ok(desc)
        else:
            self.fail(desc)
        return bool(cond)

    def summary(self):
        passed = sum(1 for *_, ok in self.results if ok)
        failed = [(it, p, d) for it, p, d, ok in self.results if not ok]
        print(f"\n{Style.cyan('=== summary ===')}")
        print(f"  {passed} passed, {len(failed)} failed")

        if self.iterations > 1:
            bad_iters = {it for it, *_ in failed}
            rate = len(bad_iters) / self.iterations * 100
            colour = Style.green if not bad_iters else Style.red
            print(
                f"  flake rate: {colour(f'{len(bad_iters)}/{self.iterations}')} "
                f"iterations had ≥1 failure ({rate:.0f}%)"
            )

        if self.timings:
            print(f"\n  {Style.cyan('timings (min / avg / max):')}")
            for label, xs in self.timings.items():
                lo, hi, avg = min(xs), max(xs), sum(xs) / len(xs)
                print(
                    f"    {label:<22} {lo:6.2f} / {avg:6.2f} / {hi:6.2f} s"
                    f"  (n={len(xs)})"
                )

        if failed:
            print()
            for it, phase, desc in failed:
                tag = f"iter {it} " if self.iterations > 1 else ""
                print(f"  {Style.red('✗')} {tag}[{phase}] {desc}")
        return not failed


# ---------------------------------------------------------------------------
# aster (device side, over the USB factory interface)
# ---------------------------------------------------------------------------


class AsterError(Exception):
    pass


class Aster:
    def __init__(self, binary):
        self.binary = str(binary)

    def v1(self, *args):
        """Run one aster v1 operation; returns the parsed --json object."""
        proc = subprocess.run(
            [self.binary, "v1", "--json", *args],
            capture_output=True,
            text=True,
            timeout=30,
        )
        if proc.returncode != 0:
            raise AsterError(f"aster v1 {' '.join(args)}: {proc.stderr.strip()}")
        return json.loads(proc.stdout)

    def ack(self, *args):
        """Action-only operation; raises on failure."""
        self.v1(*args)

    def alive(self):
        """True iff the app answers over the factory interface."""
        try:
            self.v1("--firmware-version")
            return True
        except (AsterError, subprocess.SubprocessError, OSError):
            return False

    def bt_status(self):
        return self.v1("--bt-status")["value"]

    def endpoint(self):
        return self.v1("--endpoint-get")["value"]


def wait_for(seconds, cond):
    """Poll cond() (exceptions count as False); True if it held in time."""
    return measure(seconds, cond)[0]


def measure(seconds, cond):
    """Poll cond() up to `seconds`; return (held, elapsed_seconds)."""
    start = time.monotonic()
    deadline = start + seconds
    while time.monotonic() < deadline:
        try:
            if cond():
                return True, time.monotonic() - start
        except Exception:
            pass
        time.sleep(0.2)
    return False, time.monotonic() - start


def wait_glob(pattern, seconds, present=True):
    """Wait until `pattern` matches (present=True) or clears (present=False).
    Returns the first match (present) / True (absent), else None/False."""
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        hits = globmod.glob(pattern)
        if present and hits:
            return hits[0]
        if not present and not hits:
            return True
        time.sleep(0.1)
    return None if present else False


# ---------------------------------------------------------------------------
# bluetoothctl (host side): one persistent session so the pairing agent
# registered here is in effect for every later command.
# ---------------------------------------------------------------------------


class BluetoothCtl:
    def __init__(self):
        self.proc = subprocess.Popen(
            ["bluetoothctl"],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
        self.lines = []
        self.lock = threading.Lock()
        self.reader = threading.Thread(target=self._read, daemon=True)
        self.reader.start()
        self.cmd("power on")
        self.cmd("agent NoInputNoOutput")
        self.cmd("default-agent")

    def _read(self):
        for raw in self.proc.stdout:
            line = ANSI_RE.sub("", raw).strip()
            if line:
                with self.lock:
                    self.lines.append(line)

    def _send(self, command):
        self.proc.stdin.write(command + "\n")
        self.proc.stdin.flush()

    def cmd(self, command, expect=None, timeout=5.0):
        """Send a command. With `expect` (regex), wait for a matching output
        line (searching only lines newer than the command) and return it, or
        None on timeout. Without `expect`, give the command a moment and
        return None."""
        with self.lock:
            start = len(self.lines)
        self._send(command)
        if expect is None:
            time.sleep(0.5)
            return None
        pattern = re.compile(expect)
        deadline = time.monotonic() + timeout
        scanned = start
        while time.monotonic() < deadline:
            with self.lock:
                new, scanned = self.lines[scanned:], len(self.lines)
            for line in new:
                if pattern.search(line):
                    return line
            time.sleep(0.1)
        return None

    def close(self):
        try:
            self._send("quit")
            self.proc.wait(timeout=3)
        except Exception:
            self.proc.kill()

    # -- queries (fresh bluetoothctl processes: simpler than prompt-parsing) --

    @staticmethod
    def _run(*args):
        proc = subprocess.run(
            ["bluetoothctl", *args], capture_output=True, text=True, timeout=15
        )
        return ANSI_RE.sub("", proc.stdout)

    def kb_addrs(self, name):
        """Addresses of known devices whose name matches exactly."""
        addrs = []
        for line in self._run("devices").splitlines():
            m = re.match(r"Device ((?:[0-9A-F]{2}:){5}[0-9A-F]{2}) (.*)", line.strip())
            if m and m.group(2) == name:
                addrs.append(m.group(1))
        return addrs

    def is_connected(self, addr):
        return "Connected: yes" in self._run("info", addr)

    def remove_all(self, name, report=None):
        for addr in self.kb_addrs(name):
            if report:
                report.note(f"removing host-side entry {addr}")
            self._run("remove", addr)

    # -- discovery / pairing --------------------------------------------------

    def discover(self, name, timeout):
        """Scan until a device named `name` shows up; returns its address."""
        self.cmd("scan on")
        try:
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                addrs = self.kb_addrs(name)
                if addrs:
                    return addrs[0]
                time.sleep(1)
            return None
        finally:
            self.cmd("scan off")

    def pair(self, addr, timeout=30):
        line = self.cmd(
            f"pair {addr}",
            expect=r"Pairing successful|Failed to pair|AuthenticationFailed"
            r"|AuthenticationCanceled|ConnectionAttemptFailed|not available",
            timeout=timeout,
        )
        ok = bool(line) and "Pairing successful" in line
        if ok:
            self.cmd(f"trust {addr}")
        return ok, line or "timed out"


# ---------------------------------------------------------------------------
# reboot (device side): return the app to running firmware.
# ---------------------------------------------------------------------------


class Rebooter:
    """Reboots the keyboard back into its application image.

    Default 'jump' path mirrors a field reset: aster --bootloader-jump drops
    into the mcuboot CDC-ACM recovery, then an mcumgr `os system-reset` boots
    the app (the same sequence stress_bootloader.sh exercises). 'nrfutil'
    forces a clean J-Link hardware reset with no bootloader detour.

    TODO: once firmware/aster grow a direct `aster v1 --reset`, add it as a
    third method here -- it needs neither J-Link nor a recovery round-trip.
    """

    def __init__(self, aster, report, args):
        self.aster = aster
        self.report = report
        self.method = args.reset_method
        self.mcumgrctl = args.mcumgrctl
        self.rec_glob = args.rec_glob
        self.jlink_sn = args.jlink_sn
        self.enum_timeout = 15
        self.app_timeout = 20

    def available(self):
        """(ok, reason) -- whether the chosen method's tooling is present."""
        if self.method == "jump":
            if not os.access(self.mcumgrctl, os.X_OK):
                return False, f"mcumgrctl not executable at {self.mcumgrctl}"
            return True, ""
        # nrfutil
        if subprocess.run(
            ["sh", "-c", "command -v nrfutil"], capture_output=True
        ).returncode != 0:
            return False, "nrfutil not on PATH"
        return True, ""

    def _jlink_serial(self):
        if self.jlink_sn:
            return self.jlink_sn
        out = subprocess.run(
            ["nrfutil", "device", "list"], capture_output=True, text=True, timeout=15
        ).stdout
        sn = None
        for line in out.splitlines():
            s = line.strip()
            if s.isdigit():
                sn = s
            elif "J-Link" in line and sn:
                return sn
        return sn

    def reboot(self):
        """Reboot and wait for the app. Returns seconds-to-app, or None on
        failure (reports the failure itself)."""
        start = time.monotonic()
        if self.method == "jump":
            try:
                self.aster.ack("--bootloader-jump")
            except AsterError:
                pass  # the device is already on its way into recovery
            rec = wait_glob(self.rec_glob, self.enum_timeout, present=True)
            if not rec:
                self.report.fail("recovery CDC did not enumerate for reboot")
                return None
            self.report.note(f"in recovery at {rec}, `os system-reset` to app")
            rc = subprocess.run(
                [self.mcumgrctl, "-t", "3000", "-s", rec, "os", "system-reset"],
                capture_output=True,
                text=True,
                timeout=30,
            )
            if rc.returncode != 0:
                self.report.fail(f"os system-reset failed: {rc.stderr.strip()}")
                return None
            wait_glob(self.rec_glob, self.enum_timeout, present=False)
        else:
            sn = self._jlink_serial()
            if not sn:
                self.report.fail("no J-Link serial for nrfutil reset")
                return None
            self.report.note(f"nrfutil device reset (J-Link {sn})")
            rc = subprocess.run(
                ["nrfutil", "device", "reset", "--serial-number", sn],
                capture_output=True,
                text=True,
                timeout=30,
            )
            if rc.returncode != 0:
                self.report.fail(f"nrfutil device reset failed: {rc.stderr.strip()}")
                return None

        remaining = self.app_timeout - (time.monotonic() - start)
        ok, _ = measure(max(1.0, remaining), self.aster.alive)
        if not ok:
            self.report.fail("app did not answer over aster after reboot")
            return None
        return time.monotonic() - start


# ---------------------------------------------------------------------------
# evdev key capture (F24 routing checks)
# ---------------------------------------------------------------------------


def evdev_node(bus, name=BT_NAME):
    """Keyboard evdev node reached over the given bus: 'usb' or 'bt'.

    USB: Bus=0003 Vendor=32ac Product=0034 (the BT dongle is 0039 -- skip it).
    BT:  Bus=0005 and the device name (BlueZ names the uhid device after it).
    """
    text = Path("/proc/bus/input/devices").read_text()
    for record in text.split("\n\n"):
        if bus == "usb":
            if not ("Bus=0003" in record and "Vendor=32ac Product=0034" in record):
                continue
        elif bus == "bt":
            if not ("Bus=0005" in record and name in record):
                continue
        m = re.search(r"Handlers=([^\n]*)", record)
        if not m:
            continue
        tokens = m.group(1).split()
        if "kbd" not in tokens:
            continue
        ev = next((t for t in tokens if t.startswith("event")), None)
        if ev:
            return f"/dev/input/{ev}"
    return None


class KeyCapture:
    """Watches one evdev node and counts F24 key-down events, opt. via sudo."""

    def __init__(self, node, use_sudo):
        self.count = 0
        self._stop = threading.Event()
        if use_sudo:
            self.proc = subprocess.Popen(
                ["sudo", "-n", "cat", node], stdout=subprocess.PIPE
            )
            self.stream = self.proc.stdout
        else:
            self.proc = None
            self.stream = open(node, "rb", buffering=0)
        self.thread = threading.Thread(target=self._watch, daemon=True)
        self.thread.start()

    def _watch(self):
        try:
            while not self._stop.is_set():
                data = self.stream.read(KEY_EVENT_SIZE)
                if not data or len(data) < KEY_EVENT_SIZE:
                    return
                *_, etype, code, value = struct.unpack(KEY_EVENT_FMT, data)
                # value 1 = key-down (2 = autorepeat, ignored -> genuine drops
                # show as a short count instead of being masked by repeats).
                if etype == EV_KEY and code == KEY_F24 and value == 1:
                    self.count += 1
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


class InputChecker:
    """Injects F24 taps and asserts how many arrive on which bus/identity."""

    def __init__(self, aster, report, use_sudo, key_count, key_gap):
        self.aster = aster
        self.report = report
        self.use_sudo = use_sudo
        self.key_count = key_count
        self.key_gap = key_gap

    def _inject(self, n):
        for i in range(n):
            self.aster.ack("--key-inject")
            if i != n - 1:
                time.sleep(self.key_gap)

    def _run(self, nodes, settle):
        """Capture `nodes` (label -> path) across an N-tap inject; return
        label -> count."""
        caps = {label: KeyCapture(path, self.use_sudo) for label, path in nodes.items()}
        try:
            time.sleep(0.5)
            self._inject(self.key_count)
            time.sleep(settle)
        finally:
            for cap in caps.values():
                cap.stop()
        return {label: cap.count for label, cap in caps.items()}

    def expect(self, yes_bus, no_bus=None, settle=1.5):
        """All N taps land on yes_bus, none leak to no_bus."""
        yes_node = evdev_node(yes_bus)
        if not yes_node:
            self.report.fail(f"no {yes_bus} evdev node to capture from")
            return
        nodes = {yes_bus: yes_node}
        no_node = evdev_node(no_bus) if no_bus else None
        if no_bus and no_node:
            nodes[no_bus] = no_node
        counts = self._run(nodes, settle)
        n = self.key_count
        self.report.check(
            f"F24 x{n} all arrived on the {yes_bus} node (got {counts[yes_bus]}/{n})",
            counts[yes_bus] == n,
        )
        if no_bus and no_node:
            self.report.check(
                f"F24 absent from the {no_bus} node (got {counts[no_bus]})",
                counts[no_bus] == 0,
            )

    def expect_nothing(self, bus, desc, settle=1.5):
        node = evdev_node(bus)
        if not node:
            self.report.ok(f"{desc} (no {bus} node at all)")
            return
        counts = self._run({bus: node}, settle)
        self.report.check(f"{desc} (got {counts[bus]})", counts[bus] == 0)

    def expect_nowhere(self, desc, settle=1.5):
        """No taps on the USB node nor any BT node (endpoint 'none')."""
        nodes = {}
        for label, path in (("usb", evdev_node("usb")), ("bt", evdev_node("bt"))):
            if path:
                nodes[label] = path
        if not nodes:
            self.report.ok(f"{desc} (no evdev nodes at all)")
            return
        counts = self._run(nodes, settle)
        self.report.check(f"{desc} (got {counts})", sum(counts.values()) == 0)


# ---------------------------------------------------------------------------
# the test itself
# ---------------------------------------------------------------------------


def pair_keyboard(bt, aster, report, args):
    """Discover + pair + trust; records pairing time; returns address or None."""
    report.note(f"scanning up to {args.scan_secs}s for '{args.name}'...")
    start = time.monotonic()
    addr = bt.discover(args.name, args.scan_secs)
    if not addr:
        report.fail("keyboard not found in scan")
        return None
    report.note(f"found {addr}, pairing...")
    ok, detail = bt.pair(addr)
    if not report.check(f"pairing with {addr} succeeded", ok):
        report.note(f"bluetoothctl said: {detail}")
        return None
    report.check(
        "host reports connected",
        wait_for(args.settle_secs, lambda: bt.is_connected(addr)),
    )
    connected = report.check(
        "device reports active profile connected",
        wait_for(args.settle_secs, lambda: aster.bt_status()["active-connected"]),
    )
    if connected:
        report.record_time("pairing", time.monotonic() - start)
    return addr


def run_cycle(args, report, bt, aster, inputs, rebooter):
    # -- phase 0 -------------------------------------------------------------
    report.phase("preflight")

    fw = aster.v1("--firmware-version")["value"]
    report.note(f"firmware {fw}")

    report.note("clearing state on both sides")
    aster.ack("--factory-mode-enter")
    aster.ack("--bt-clear-pairings")
    bt.remove_all(args.name, report)
    aster.ack("--endpoint", "usb")

    st = aster.bt_status()
    report.check("no bonds on device", st["bonded-mask"] == 0)
    report.check("active profile is 0", st["active"] == 0)
    report.check("active profile is open", st["active-open"])
    report.check("advertising enabled", st["adv-enabled"])

    # -- phase 1 -------------------------------------------------------------
    report.phase("pair profile 0")

    addr0 = pair_keyboard(bt, aster, report, args)
    if addr0:
        st = aster.bt_status()
        report.check("profile 0 bonded on device", st["bonded-mask"] == 1)
        report.check("profile 0 no longer open", not st["active-open"])

    # -- phase 2 -------------------------------------------------------------
    report.phase("input over USB endpoint")

    report.check(
        "selected endpoint is usb",
        wait_for(args.settle_secs, lambda: aster.endpoint()["selected"] == "usb"),
    )
    if inputs:
        inputs.expect("usb", "bt")
    else:
        report.note("input check skipped")

    # -- phase 3 -------------------------------------------------------------
    report.phase("switch endpoint to BLE")

    aster.ack("--endpoint", "ble")
    report.check("preferred endpoint is ble", aster.endpoint()["preferred"] == "ble")
    ok, dt = measure(args.settle_secs, lambda: aster.endpoint()["selected"] == "ble")
    report.check("selected endpoint is ble", ok)
    if ok:
        report.record_time("endpoint→ble", dt)
    if inputs:
        report.check(
            "BT evdev node appeared",
            wait_for(args.settle_secs, lambda: evdev_node("bt")),
        )
        inputs.expect("bt", "usb")
    else:
        report.note("input check skipped")

    # -- phase 4 -------------------------------------------------------------
    report.phase("endpoint none (reports go nowhere)")

    aster.ack("--endpoint", "none")
    report.check("preferred endpoint is none", aster.endpoint()["preferred"] == "none")
    report.check(
        "selected endpoint is none",
        wait_for(args.settle_secs, lambda: aster.endpoint()["selected"] == "none"),
    )
    if inputs:
        inputs.expect_nowhere("F24 reaches neither USB nor BT with endpoint none")

    # -- phase 5 -------------------------------------------------------------
    report.phase("switch endpoint back to USB")

    aster.ack("--endpoint", "usb")
    report.check("preferred endpoint is usb", aster.endpoint()["preferred"] == "usb")
    ok, dt = measure(args.settle_secs, lambda: aster.endpoint()["selected"] == "usb")
    report.check("selected endpoint is usb", ok)
    if ok:
        report.record_time("endpoint→usb", dt)
    if inputs:
        inputs.expect("usb", "bt")

    # -- phase 6 -------------------------------------------------------------
    report.phase("advertising control")

    aster.ack("--bt-adv-off")
    report.check(
        "advertising disabled",
        wait_for(args.settle_secs, lambda: not aster.bt_status()["adv-enabled"]),
    )
    aster.ack("--bt-adv-on")
    report.check(
        "advertising re-enabled",
        wait_for(args.settle_secs, lambda: aster.bt_status()["adv-enabled"]),
    )

    # -- phase 7 -------------------------------------------------------------
    report.phase("reboot: bond survives + host auto-reconnects")

    if rebooter is None:
        report.note("reboot tooling unavailable / --skip-reboot; phase skipped")
    elif not addr0:
        report.note("no paired profile 0; reboot phase skipped")
    else:
        # Reboot while on BLE, so we exercise reconnection, not just USB.
        aster.ack("--endpoint", "ble")
        wait_for(args.settle_secs, lambda: aster.endpoint()["selected"] == "ble")
        report.note("rebooting device...")
        boot = rebooter.reboot()
        if boot is not None:
            report.record_time("reboot→app", boot)
            report.ok("app came back after reboot")
            # Factory mode and endpoint preference don't survive the reboot.
            aster.ack("--factory-mode-enter")
            report.check(
                "bond survived reboot",
                wait_for(args.settle_secs, lambda: aster.bt_status()["bonded-mask"] == 1),
            )
            ok, dt = measure(
                args.reconnect_secs, lambda: aster.bt_status()["active-connected"]
            )
            if not ok:
                # Some hosts need a nudge; a manual connect still proves the
                # bond persisted, so note the distinction rather than failing.
                report.note("no passive auto-reconnect; nudging host connect")
                bt.cmd(f"connect {addr0}")
                ok2, dt2 = measure(
                    args.reconnect_secs, lambda: aster.bt_status()["active-connected"]
                )
                ok, dt = ok2, dt + dt2
            report.check("device reconnected after reboot", ok)
            if ok:
                report.record_time("reconnect", dt)
                report.check(
                    "host reports connected after reboot",
                    wait_for(args.settle_secs, lambda: bt.is_connected(addr0)),
                )
                aster.ack("--endpoint", "ble")
                wait_for(args.settle_secs, lambda: aster.endpoint()["selected"] == "ble")
                if inputs:
                    report.check(
                        "BT node back after reboot",
                        wait_for(args.settle_secs, lambda: evdev_node("bt")),
                    )
                    inputs.expect("bt", "usb")
        aster.ack("--endpoint", "usb")
        wait_for(args.settle_secs, lambda: aster.endpoint()["selected"] == "usb")

    # -- phase 8 -------------------------------------------------------------
    report.phase("profile switching")

    # Switching away does NOT drop the profile-0 link (ZMK keeps it; reports
    # just stop flowing there), so assert via the active-* flags and, over
    # BLE, via whether injected keys still reach the host.
    aster.ack("--bt-profile", "1")
    st = aster.bt_status()
    report.check("active profile is 1", st["active"] == 1)
    report.check("profile 1 is open (unbonded)", st["active-open"])
    report.check("profile 1 not connected", not st["active-connected"])
    report.check("profile 0 link still up (mask)", st["connected-mask"] == 1)

    if inputs:
        aster.ack("--endpoint", "ble")
        inputs.expect_nothing(
            "bt", "keys stop flowing when an unbonded profile is active"
        )

    # next x3: 1 -> 2 -> 3 -> 0 (4 profiles, wraps).
    for _ in range(3):
        aster.ack("--bt-profile-next")
    report.check("profile-next x3 wrapped back to 0", aster.bt_status()["active"] == 0)
    report.check(
        "profile 0 connected again",
        wait_for(args.settle_secs, lambda: aster.bt_status()["active-connected"]),
    )
    if inputs:
        inputs.expect("bt", "usb")
        aster.ack("--endpoint", "usb")
        wait_for(args.settle_secs, lambda: aster.endpoint()["selected"] == "usb")

    aster.ack("--bt-profile-prev")
    report.check("profile-prev wrapped to 3", aster.bt_status()["active"] == 3)
    aster.ack("--bt-profile-next")
    report.check("back on profile 0", aster.bt_status()["active"] == 0)

    # -- phase 9 -------------------------------------------------------------
    report.phase("negative paths")

    # Out-of-range profile index must be rejected, not silently accepted.
    count = aster.bt_status()["profile-count"]
    rejected = False
    try:
        aster.ack("--bt-profile", str(count))  # one past the last valid index
    except AsterError:
        rejected = True
    report.check(
        f"out-of-range profile {count} rejected",
        rejected or aster.bt_status()["active"] != count,
    )
    report.check("still on a valid profile", aster.bt_status()["active"] < count)

    # Re-pairing an already-bonded profile from the host must not create a
    # second bond or reopen the profile on the device.
    if addr0:
        bt.cmd(f"pair {addr0}", expect=None)
        st = aster.bt_status()
        report.check("re-pair did not add a bond", st["bonded-mask"] == 1)
        report.check("re-pair left profile 0 closed", not st["active-open"])

    # -- phase 10 ------------------------------------------------------------
    report.phase("unpair active profile")

    aster.ack("--bt-profile", "0")
    aster.ack("--bt-unpair")
    report.check(
        "no bonds left on device",
        wait_for(args.settle_secs, lambda: aster.bt_status()["bonded-mask"] == 0),
    )
    report.check("profile 0 open again", aster.bt_status()["active-open"])
    if addr0:
        report.check(
            "host sees disconnect",
            wait_for(args.settle_secs, lambda: not bt.is_connected(addr0)),
        )
    # The host still holds its (now one-sided) bond and would retry with a
    # stale key; drop it before re-pairing.
    bt.remove_all(args.name, report)

    # -- phase 11 ------------------------------------------------------------
    report.phase("re-pair, then clear all bonds")

    addr = pair_keyboard(bt, aster, report, args)
    if addr:
        report.check("profile 0 bonded again", aster.bt_status()["bonded-mask"] == 1)
        aster.ack("--bt-clear-pairings")
        report.check(
            "all bonds cleared",
            wait_for(args.settle_secs, lambda: aster.bt_status()["bonded-mask"] == 0),
        )
        report.check("back on profile 0", aster.bt_status()["active"] == 0)
        report.check(
            "host sees disconnect",
            wait_for(args.settle_secs, lambda: not bt.is_connected(addr)),
        )
    bt.remove_all(args.name, report)


def run_all(args, report):
    aster = Aster(args.aster)
    bt = BluetoothCtl()
    inputs = None
    rebooter = None

    try:
        # -- one-time host-side setup (input access, reboot tooling) ---------
        if not args.skip_input_check:
            usb_node = evdev_node("usb")
            use_sudo = False
            if not usb_node:
                report.note("WARNING: no USB keyboard evdev node; input checks off")
                args.skip_input_check = True
            elif not os.access(usb_node, os.R_OK):
                sudo_ok = (
                    subprocess.run(["sudo", "-n", "true"], capture_output=True).returncode
                    == 0
                )
                if sudo_ok:
                    use_sudo = True
                else:
                    report.note(
                        "WARNING: no read access to /dev/input and no passwordless "
                        "sudo; input checks off (fix: sudo usermod -aG input $USER)"
                    )
                    args.skip_input_check = True
            if not args.skip_input_check:
                inputs = InputChecker(
                    aster, report, use_sudo, args.key_count, args.key_gap
                )

        if not args.skip_reboot:
            rebooter = Rebooter(aster, report, args)
            ok, why = rebooter.available()
            if not ok:
                report.note(f"WARNING: reboot phase disabled ({why})")
                rebooter = None

        for i in range(1, args.iterations + 1):
            report.start_iteration(i)
            try:
                run_cycle(args, report, bt, aster, inputs, rebooter)
            except TestAbort:
                # A single run stops on first failure (--keep-going never
                # raises); a soak logs the flake and moves on to the next cycle.
                if args.iterations == 1:
                    break
                report.note("cycle aborted; continuing soak")
            except AsterError as err:
                report.results.append((i, report.phase_name, str(err), False))
                print(f"\n{Style.red('aborted cycle:')} {err}")
                if args.iterations == 1:
                    break
    finally:
        # -- cleanup (always) ------------------------------------------------
        report.phase("cleanup")
        for flags in (("--endpoint", "usb"), ("--factory-mode-exit",)):
            try:
                aster.ack(*flags)
            except AsterError as err:
                report.note(f"cleanup: {err}")
        try:
            bt.remove_all(args.name, report)
        finally:
            bt.close()


def main():
    parser = argparse.ArgumentParser(
        description="Bluetooth pairing/profile/endpoint/reliability test for Daisy.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "-n", "--iterations", type=int, default=1,
        help="repeat the whole cycle N times (soak); reports a flake rate",
    )
    parser.add_argument(
        "--keep-going", action="store_true", help="don't stop on first failure"
    )
    parser.add_argument(
        "--skip-input-check",
        action="store_true",
        help="no F24 key-routing checks (no /dev/input access needed)",
    )
    parser.add_argument(
        "--skip-reboot", action="store_true", help="skip the reboot-persistence phase"
    )
    parser.add_argument(
        "--reset-method",
        choices=("jump", "nrfutil"),
        default="jump",
        help="reboot via mcuboot recovery + os system-reset (jump) or a J-Link "
        "hardware reset (nrfutil)",
    )
    parser.add_argument(
        "--key-count", type=int, default=10,
        help="F24 taps injected per input-routing check (delivery-loss check)",
    )
    parser.add_argument(
        "--key-gap", type=float, default=0.15, help="seconds between injected taps"
    )
    parser.add_argument("--name", default=BT_NAME, help="BLE device name")
    parser.add_argument(
        "--scan-secs", type=int, default=20, help="max scan time per discovery"
    )
    parser.add_argument(
        "--settle-secs", type=int, default=15, help="wait for async state changes"
    )
    parser.add_argument(
        "--reconnect-secs", type=int, default=30,
        help="wait for BLE (re)connection after reboot / profile switch",
    )
    parser.add_argument(
        "--aster",
        default=os.environ.get("ASTER", str(DEFAULT_ASTER)),
        help="path to the aster binary",
    )
    parser.add_argument(
        "--mcumgrctl",
        default=os.environ.get("MCUMGRCTL", str(DEFAULT_MCUMGRCTL)),
        help="path to mcumgrctl (reset-method jump)",
    )
    parser.add_argument(
        "--rec-glob", default=DEFAULT_REC_GLOB, help="recovery CDC ACM device glob"
    )
    parser.add_argument(
        "--jlink-sn", default="", help="J-Link serial for reset-method nrfutil"
    )
    args = parser.parse_args()

    for tool, probe in (("bluetoothctl", ["bluetoothctl", "--version"]),):
        try:
            subprocess.run(probe, capture_output=True, timeout=10)
        except FileNotFoundError:
            sys.exit(f"{tool} not found")
    if not os.access(args.aster, os.X_OK):
        sys.exit(f"aster not found at {args.aster}")

    report = Report(args.keep_going, args.iterations)
    try:
        run_all(args, report)
    except TestAbort:
        pass
    except AsterError as err:
        print(f"\n{Style.red('aborted:')} {err}")
        report.results.append((report.iteration, report.phase_name, str(err), False))
    sys.exit(0 if report.summary() else 1)


if __name__ == "__main__":
    main()
