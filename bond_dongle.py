#!/usr/bin/env python3
"""Bond a Daisy keyboard to exactly one BLE central: the hid-remapper dongle
(default) or this laptop (--laptop).

The keyboard is identified over USB (aster), which is the disambiguator: even
with many keyboards advertising nearby, we read *this* keyboard's exact BLE
address and target only it. `aster --bt-address` reports both the static
identity and the address advertised right now.

  address-pin (default, ZMK privacy OFF)
      Pin the static identity address (permanent, FICR-derived).

  --privacy (ZMK privacy ON, rotating RPA)
      The static identity is NOT what's on air, so instead we open the keyboard
      profile, read the address it is advertising *right now* (the current RPA),
      and pin that. The pair completes in seconds, far under BT_RPA_TIMEOUT
      (900 s), so the RPA is stable long enough. After bonding, the central has
      the keyboard's IRK and resolves later RPA rotations to that identity.

Targets:
  (default)   the hid-remapper dongle over USB (config-tool PAIR_WITH_ADDRESS)
  --laptop    this host's BlueZ adapter (bluetoothctl pair/trust/connect)

The keyboard's target profile must advertise open (no bond) for a new pair;
--prep (implied by --privacy) unpairs the active profile and enables adv first.

Env overrides: ASTER, CONFIG_TOOL.
"""

import argparse
import json
import os
import re
import shutil
import subprocess
import sys
import threading
import time

ASTER = os.environ.get("ASTER", os.path.expanduser("~/clone/aster/target/debug/aster"))
CONFIG_TOOL = os.environ.get(
    "CONFIG_TOOL", os.path.expanduser("~/clone/hid-remapper-private/config-tool")
)


def run(cmd, **kwargs):
    """Run a command, echoing it, and raise on failure."""
    print("+ " + " ".join(cmd), file=sys.stderr)
    return subprocess.run(cmd, check=True, **kwargs)


def aster(args, kb_device, capture=False):
    """Invoke `aster v1 ...`, optionally targeting a specific device."""
    cmd = [ASTER, "v1"]
    if kb_device:
        cmd += ["--device", kb_device]
    cmd += args
    if capture:
        return run(cmd, capture_output=True, text=True)
    return run(cmd)


def read_kb_address(kb_device):
    """Return the keyboard's address info dict (identity / current / privacy)."""
    proc = aster(["--json", "--bt-address"], kb_device, capture=True)
    return json.loads(proc.stdout)["value"]


def open_keyboard_profile(kb_device):
    """Drop the active profile's bond and enable advertising so it is open."""
    print("preparing keyboard: unpair active profile + advertising on")
    aster(["--bt-unpair"], kb_device)
    aster(["--bt-adv-on"], kb_device)


def dongle_pair(addr, addr_type):
    """Tell the dongle to targeted-pair with exactly one address."""
    print(f"telling dongle to pair with {addr} ({addr_type}) ...")
    run([sys.executable, "control_bonds.py", "pair", addr, addr_type], cwd=CONFIG_TOOL)


_ANSI_RE = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")


class BluetoothCtl:
    """Minimal persistent bluetoothctl session. One session keeps the pairing
    agent registered (a one-shot `bluetoothctl pair` has no agent and fails)."""

    def __init__(self, agent_cap="NoInputNoOutput"):
        self.proc = subprocess.Popen(
            ["bluetoothctl"],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
        self.lines = []
        self.lock = threading.Lock()
        threading.Thread(target=self._read, daemon=True).start()
        self.cmd("power on")
        # bluetoothctl auto-registers a default (KeyboardDisplay) agent at
        # startup; drop it first, otherwise `agent <cap>` is rejected with
        # "Agent is already registered" and our capability never takes effect.
        # Agent capability decides the pairing method: NoInputNoOutput -> Just
        # Works; DisplayYesNo -> the host displays the passkey (needed to relay
        # it for an MITM/passkey-entry keyboard).
        self.cmd("agent off")
        self.cmd(f"agent {agent_cap}")
        self.cmd("default-agent")

    def _read(self):
        for raw in self.proc.stdout:
            line = _ANSI_RE.sub("", raw).strip()
            if line:
                with self.lock:
                    self.lines.append(line)

    def send(self, command):
        """Send a command; return a cursor into the output for expect_from()."""
        with self.lock:
            start = len(self.lines)
        self.proc.stdin.write(command + "\n")
        self.proc.stdin.flush()
        return start

    def expect_from(self, start, pattern, timeout):
        """Wait for a line matching `pattern` among output produced since the
        `start` cursor, returning it or None on timeout."""
        p = re.compile(pattern)
        deadline = time.monotonic() + timeout
        scanned = start
        while time.monotonic() < deadline:
            with self.lock:
                new, scanned = self.lines[scanned:], len(self.lines)
            for line in new:
                if p.search(line):
                    return line
            time.sleep(0.1)
        return None

    def cmd(self, command, expect=None, timeout=5.0):
        """Send a command; with `expect` (regex) wait for a matching output line
        (only lines newer than the command), returning it or None on timeout."""
        start = self.send(command)
        if expect is None:
            time.sleep(0.5)
            return None
        return self.expect_from(start, expect, timeout)

    def close(self):
        try:
            self.proc.stdin.write("quit\n")
            self.proc.stdin.flush()
            self.proc.wait(timeout=3)
        except Exception:
            self.proc.kill()

    @staticmethod
    def _run(*args):
        proc = subprocess.run(
            ["bluetoothctl", *args], capture_output=True, text=True, timeout=15
        )
        return _ANSI_RE.sub("", proc.stdout)

    def seen(self, addr):
        """True once BlueZ has discovered `addr` (case-insensitive)."""
        want = addr.upper()
        for line in self._run("devices").splitlines():
            m = re.match(r"Device ((?:[0-9A-F]{2}:){5}[0-9A-F]{2})", line.strip())
            if m and m.group(1).upper() == want:
                return True
        return False

    def discover(self, addr, timeout=30):
        self.cmd("scan on")
        try:
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                if self.seen(addr):
                    return True
                time.sleep(1)
            return False
        finally:
            self.cmd("scan off")


def bluez_bond_state(addr):
    """Return (paired, connected) booleans for `addr` per `bluetoothctl info`."""
    out = BluetoothCtl._run("info", addr)
    paired = re.search(r"Paired:\s*yes", out) is not None
    connected = re.search(r"Connected:\s*yes", out) is not None
    return paired, connected


def remove_bond(addr, timeout=10):
    """Untrust + remove a bond and wait until BlueZ reports it gone.

    Untrust first: a trusted + wake-allowed device auto-reconnects the instant
    it advertises, recreating the bond between `remove` and the next `pair`
    (which then fails AlreadyExists). Polling `info` until Paired:no closes that
    race far more reliably than a blind sleep. Returns True if the bond cleared."""
    BluetoothCtl._run("untrust", addr)
    deadline = time.monotonic() + timeout
    while True:
        BluetoothCtl._run("remove", addr)
        time.sleep(0.5)
        paired, _ = bluez_bond_state(addr)
        if not paired:
            return True
        if time.monotonic() >= deadline:
            return False


_PASSKEY_RE = r"Passkey:?\s*0*(\d{1,6})"
_PAIR_RESULT_RE = (
    r"Pairing successful|Failed to pair|AuthenticationFailed"
    r"|AuthenticationCanceled|ConnectionAttemptFailed|not available"
)


def laptop_pair(addr, addr_type, relay=None, refresh=None, trust_addr=None,
                manual=False, repair=False, attempts=3):
    """Pair the keyboard to this host's BlueZ adapter (pair + trust + connect).

    If a bond already exists for this keyboard, `pair` returns AlreadyExists and
    can never succeed. By default we treat that as "already bonded" and just
    reconnect (trust + connect). With `repair=True` we tear the bond down
    (untrust + remove) and pair fresh -- use it after re-flashing or when the
    host and keyboard bond state have drifted apart.

    Uses a DisplayYesNo agent so the host displays a passkey if the keyboard
    demands authenticated (MITM) pairing. In auto mode the displayed passkey is
    relayed to the keyboard over USB via `relay(passkey)` (MITM, hands-free); in
    `manual` mode it is shown for a human to type on the Daisy, with a long
    timeout. A Just Works keyboard shows no passkey and pairs directly.

    Retries a few times: re-pairing right after --bt-unpair can race the host's
    stale-LTK reconnect attempts (AuthenticationFailed / ConnectionAttemptFailed
    on the first try); settling and re-discovering clears it. `refresh` (if
    given) returns a fresh (addr, addr_type) before each retry -- needed in
    privacy mode, where the keyboard rotates its RPA after a failed attempt."""
    if not shutil.which("bluetoothctl"):
        raise RuntimeError("bluetoothctl not found (install bluez)")
    # BlueZ keys the bond by the resolved identity, not the (maybe private)
    # address we pair with, so query/tear down using the identity when known.
    key_addr = trust_addr or addr
    ctl = BluetoothCtl(agent_cap="DisplayYesNo")
    try:
        paired, connected = bluez_bond_state(key_addr)
        if paired and not repair:
            print(f"{key_addr} is already bonded to this laptop; reconnecting "
                  f"(pass --repair to tear it down and pair fresh) ...")
            ctl.cmd(f"trust {key_addr}")
            if connected:
                print(f"already connected to {key_addr}")
                return
            line = ctl.cmd(f"connect {key_addr}",
                           expect=r"Connection successful|Failed to connect",
                           timeout=15)
            if line and "Connection successful" in line:
                print(f"connected to {key_addr}")
                return
            raise RuntimeError(
                f"bonded but could not connect ({line or 'timed out'}); is the "
                f"keyboard on the profile paired to this host and advertising? "
                f"re-run with --repair to pair fresh")
        if paired:  # repair: drop the stale bond before we try to pair
            print(f"--repair: removing existing bond for {key_addr} ...")
            if not remove_bond(key_addr):
                print(f"  warning: bond for {key_addr} did not clear; "
                      f"pairing may still fail AlreadyExists", file=sys.stderr)
        last = "no attempt"
        for attempt in range(1, attempts + 1):
            if attempt > 1:
                # Fresh start: drop any half-bond, let stale reconnects settle.
                remove_bond(key_addr)
                print(f"retry {attempt}/{attempts} after settle ...")
                time.sleep(3)
                if refresh is not None:
                    addr, addr_type = refresh()
                    print(f"refreshed target address: {addr} ({addr_type})")
            print(f"scanning for {addr} ...")
            if not ctl.discover(addr):
                last = f"{addr} not seen while scanning (is it advertising open?)"
                print(f"  attempt {attempt}: {last}", file=sys.stderr)
                continue
            print(f"pairing with {addr} (attempt {attempt}) ...")
            start = ctl.send(f"pair {addr}")
            # The host may display a passkey (MITM/passkey-entry keyboard) before
            # pairing completes. In auto mode we relay it over USB; in manual
            # mode we show it and wait for a human to type it on the Daisy (with
            # a long timeout -- unlike GNOME, which cancels too fast to type).
            line = ctl.expect_from(start, _PASSKEY_RE + "|" + _PAIR_RESULT_RE, 30)
            m = re.search(_PASSKEY_RE, line or "")
            if m:
                passkey = int(m.group(1))
                if manual:
                    print(f"\n  >>> On the Daisy, type:  {passkey:06d}  then press Enter"
                          f"  (Esc cancels)  <<<\n")
                    line = ctl.expect_from(start, _PAIR_RESULT_RE, 120)
                else:
                    print(f"host displayed passkey {passkey:06d}; relaying to keyboard ...")
                    if relay is None:
                        raise RuntimeError("passkey requested but no relay configured")
                    relay(passkey)
                    line = ctl.expect_from(start, _PAIR_RESULT_RE, 30)
            if line and "Pairing successful" in line:
                # After bonding BlueZ keys the device by its resolved identity,
                # not the (possibly private) address we paired with -- trust that
                # so the bond is remembered/auto-authorized.
                ctl.cmd(f"trust {trust_addr or addr}")
                ctl.cmd(f"connect {trust_addr or addr}",
                        expect=r"Connection successful|Failed to connect", timeout=15)
                print(f"paired + trusted {addr} with this laptop")
                return
            last = line or "timed out"
            print(f"  attempt {attempt} failed: {last}", file=sys.stderr)
        raise RuntimeError(f"pair failed after {attempts} attempts: {last}")
    finally:
        ctl.close()


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "--laptop",
        action="store_true",
        help="pair with this host's BlueZ adapter instead of the dongle",
    )
    parser.add_argument(
        "--manual",
        action="store_true",
        help="--laptop only: show the passkey and wait for you to type it on the "
        "Daisy (long timeout), instead of relaying it over USB",
    )
    parser.add_argument(
        "--repair",
        action="store_true",
        help="--laptop only: tear down any existing bond (untrust + remove) and "
        "pair fresh; without it, an existing bond is reused (trust + connect)",
    )
    parser.add_argument(
        "--privacy",
        action="store_true",
        help="pin the live RPA instead of the static identity (ZMK privacy ON)",
    )
    parser.add_argument(
        "--prep",
        action="store_true",
        help="unpair the active profile + enable advertising first "
        "(implied by --privacy)",
    )
    parser.add_argument(
        "--kb-device",
        metavar="SELECTOR",
        help="aster --device selector for the keyboard (default: autodetect)",
    )
    args = parser.parse_args()
    if args.manual and not args.laptop:
        parser.error("--manual only applies with --laptop")
    if args.repair and not args.laptop:
        parser.error("--repair only applies with --laptop")

    target = "laptop" if args.laptop else "dongle"

    if args.privacy:
        # Rotating RPA: open the keyboard first so it advertises the RPA, then
        # read and pin whatever it is advertising right now.
        open_keyboard_profile(args.kb_device)
        info = read_kb_address(args.kb_device)
        if not info["privacy"]:
            print(
                "note: keyboard reports privacy OFF; 'current' == identity "
                "(still fine to pin).",
                file=sys.stderr,
            )
        print(f"keyboard identity: {info['identity']} ({info['identity-type']})")
        addr, addr_type = info["current"], info["current-type"]
    else:
        # Static identity: pin the permanent address.
        info = read_kb_address(args.kb_device)
        if info["privacy"]:
            print(
                "warning: keyboard has privacy ON -- the static identity is NOT "
                "what it advertises; the central won't find it. Use --privacy.",
                file=sys.stderr,
            )
        if args.prep:
            open_keyboard_profile(args.kb_device)
        addr, addr_type = info["identity"], info["identity-type"]

    print(f"keyboard BLE address: {addr} ({addr_type}); target: {target}")
    if target == "laptop":
        # In privacy mode the RPA rotates after a failed attempt, so give the
        # retry loop a way to re-read the current live address.
        def refresh_current():
            info = read_kb_address(args.kb_device)
            return info["current"], info["current-type"]

        # Relay a host-displayed passkey to the keyboard over USB (MITM,
        # hands-free). The keyboard may take a moment to reach the passkey-entry
        # state after the host connects, so retry briefly.
        def relay_passkey(passkey):
            for _ in range(15):
                try:
                    aster(["--bt-passkey", str(passkey)], args.kb_device)
                    return
                except subprocess.CalledProcessError:
                    time.sleep(0.3)
            raise RuntimeError("keyboard did not accept the passkey (not awaiting one?)")

        laptop_pair(
            addr,
            addr_type,
            relay=None if args.manual else relay_passkey,
            refresh=refresh_current if args.privacy else None,
            trust_addr=info["identity"],
            manual=args.manual,
            repair=args.repair,
        )
    else:
        dongle_pair(addr, addr_type)


if __name__ == "__main__":
    try:
        main()
    except subprocess.CalledProcessError as e:
        print(f"error: command failed (exit {e.returncode})", file=sys.stderr)
        sys.exit(1)
