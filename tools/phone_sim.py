#!/usr/bin/env python3
"""phone_sim.py - a Python stand-in for the phone brain (prompt §4.7).

Bench-tests the vacuum_phone_esp32 firmware BEFORE the app exists:
sends HELO to claim the board, prints HBT2/LDS2 at a glance, drives with
HBC2 from the keyboard, measures PNG1/PNG2 RTT, latches/clears E-stop.

Stdlib only. Usage:

    python3 tools/phone_sim.py 192.168.1.42          # board IP
    python3 tools/phone_sim.py hoverboard.local      # or mDNS name
    python3 tools/phone_sim.py <ip> --takeover       # steal a live owner
    python3 tools/phone_sim.py <ip> --no-drive       # read-only monitor

Keys while running:
    w/s   forward / back        a/d   turn left / right
    x     stop (zero RPM)       space same
    e     latch E-stop          c     clear E-stop
    p     one PNG1 ping         h     re-send HELO
    q     quit (sends nothing; the board's lease expires in 3 s)

Protocol reference: docs/PROTOCOL.md. This file is also the source of the
captured packets the app's decoder unit tests are built from (save a session
with --capture FILE).
"""

import argparse
import math
import select
import socket
import struct
import sys
import termios
import time
import tty

# ---------------------------------------------------------------- wire format
CMD_PORT_DEFAULT = 9901   # HELO / HBC2 / PNG1 land here on the board
TELEM_PORT_DEFAULT = 9902 # HBT2 / BYE1 come back here
SCAN_PORT_DEFAULT = 9900  # LDS2 comes back here
PROTO = 2

HELO_TAKEOVER = 0x0001
HBC2_ENABLE = 0x0001
HBC2_BUZZER = 0x0002
HBC2_ESTOP = 0x0004
HBC2_ESTOP_CLEAR = 0x0008

STATUS_NAMES = {0: "GRANTED", 1: "DENIED", 2: "VERSION_MISMATCH"}


def u16(b, off):
    return struct.unpack_from("<H", b, off)[0]


def i16(b, off):
    return struct.unpack_from("<h", b, off)[0]


def u32(b, off):
    return struct.unpack_from("<I", b, off)[0]


# -------------------------------------------------------------------- decode
def describe_hbt2(d):
    """One-line summary of a 96-byte HBT2 (docs/PROTOCOL.md §3.6)."""
    if len(d) < 96 or d[:4] != b"HBT2":
        return None
    seq = u16(d, 4)
    ms = u32(d, 6)
    flags = u16(d, 10)
    ticks_l, ticks_r = i32(d, 14), i32(d, 18)
    rpm_l, rpm_r = i16(d, 22), i16(d, 24)
    batt = i16(d, 26) / 100.0
    sonar = u16(d, 42)
    cmd_seq = u16(d, 78)
    cmd_age = u16(d, 84)
    link = d[86]
    rssi = struct.unpack_from("<b", d, 87)[0]
    reflex = d[88]
    lidar_hz = d[89] / 10.0
    out_l, out_r = i16(d, 90) / 10.0, i16(d, 92) / 10.0
    crc = u16(d, 94)
    estop = bool(link & 0x01)
    return (
        f"HBT2 #{seq} board {ms} ms  batt {batt:5.2f} V  "
        f"ticks {ticks_l}/{ticks_r}  rpm {rpm_l:+4.0f}/{rpm_r:+4.0f}  "
        f"out {out_l:+5.1f}/{out_r:+5.1f}  sonar {sonar:4} mm  "
        f"cmd #{cmd_seq} age {cmd_age:4} ms  rssi {rssi:4} dBm  "
        f"lidar {lidar_hz:4.1f} Hz crc {crc}  "
        f"{'E-STOP ' if estop else ''}"
        f"{'MB-DOWN ' if not (flags & 1) else ''}"
        f"{'TIMEOUT ' if flags & 2 else ''}"
        f"reflex={reflex}"
    )


def i32(b, off):
    return struct.unpack_from("<i", b, off)[0]


def describe_lds2(d):
    """One-line summary of an LDS2 datagram (docs/PROTOCOL.md §3.7)."""
    if len(d) < 26 or d[:4] != b"LDS2":
        return None
    bins = u16(d, 4)
    scan_id = u32(d, 6)
    start_ms, end_ms = u32(d, 10), u32(d, 14)
    centi_hz = u16(d, 18)
    points, valid = u16(d, 20), u16(d, 22)
    flags = u16(d, 24)
    need = 26 + bins * 2 + (bins if flags & 1 else 0)
    if len(d) < need:
        return f"LDS2 #{scan_id} TRUNCATED {len(d)}/{need}"
    ranges = struct.unpack_from(f"<{bins}H", d, 26)
    nonzero = [r for r in ranges if r]
    near = min(nonzero) if nonzero else 0
    far = max(nonzero) if nonzero else 0
    return (
        f"LDS2 #{scan_id} {bins} bins {centi_hz / 100.0:4.2f} Hz  "
        f"{points} pts ({valid} valid)  span {end_ms - start_ms} ms  "
        f"range {near}-{far} mm"
    )


def describe_hack(d):
    if len(d) < 20 or d[:4] != b"HACK":
        return None
    proto, status, board = u16(d, 4), d[6], d[7]
    bins, tpr = u16(d, 8), u16(d, 10)
    session, board_ms = u32(d, 12), u32(d, 16)
    fw = d[20:].split(b"\0", 1)[0].decode(errors="replace")
    board_name = "ESP32-S3" if board == 1 else "ESP32-WROOM"
    return (
        f"HACK {STATUS_NAMES.get(status, status)}  proto {proto}  {board_name}  "
        f"{bins} bins  {tpr} ticks/rev  session {session:08x}  "
        f"board {board_ms} ms  fw {fw!r}"
    )


def describe_bye(d):
    if len(d) < 5 or d[:4] != b"BYE1":
        return None
    reason = {0: "released", 1: "lease expired", 2: "takeover"}.get(d[4], d[4])
    return f"BYE1 ({reason})"


# --------------------------------------------------------------------- drive
class Driver:
    """Converts held keys into HBC2 wheel RPM targets (+ = forward)."""

    LINEAR_RPM = 30.0    # ~0.30 m/s on a 96.15 mm wheel
    ANGULAR_RPM = 25.0   # per-wheel differential for a turn

    def __init__(self):
        self.forward = 0
        self.turn = 0
        self.estop = False
        self.buzzer = False

    def rpm(self):
        if self.estop:
            return 0.0, 0.0, HBC2_ESTOP | HBC2_ENABLE
        left = self.forward * self.LINEAR_RPM - self.turn * self.ANGULAR_RPM
        right = self.forward * self.LINEAR_RPM + self.turn * self.ANGULAR_RPM
        flags = HBC2_ENABLE
        if self.buzzer:
            flags |= HBC2_BUZZER
        return left, right, flags

    def hbc2(self, seq, phone_ms):
        left, right, flags = self.rpm()
        # rpm x10 as i16 (docs/PROTOCOL.md §3.4)
        return struct.pack(
            "<4sHhhHI", b"HBC2", seq & 0xFFFF,
            int(round(left * 10)), int(round(right * 10)), flags, phone_ms & 0xFFFFFFFF
        )


# ---------------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("host", help="board IP or mDNS name (hoverboard.local)")
    ap.add_argument("--cmd-port", type=int, default=CMD_PORT_DEFAULT)
    ap.add_argument("--telem-port", type=int, default=TELEM_PORT_DEFAULT)
    ap.add_argument("--scan-port", type=int, default=SCAN_PORT_DEFAULT)
    ap.add_argument("--takeover", action="store_true",
                    help="steal ownership from another phone")
    ap.add_argument("--no-drive", action="store_true",
                    help="monitor only; never send HBC2")
    ap.add_argument("--capture", metavar="FILE",
                    help="append every received datagram (hex) to FILE")
    args = ap.parse_args()

    # Sockets: one for the command channel (HELO/HBC2/PNG1 -> board:9901,
    # replies HBT2/PNG2/BYE1 come back to our source port), one bound to the
    # announced scan port for LDS2.
    cmd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    cmd.settimeout(2.0)
    cmd.bind(("0.0.0.0", 0))
    scan = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    scan.bind(("0.0.0.0", args.scan_port))
    scan.setblocking(False)

    board = (socket.gethostbyname(args.host), args.cmd_port)
    my_cmd_port = cmd.getsockname()[1]
    print(f"phone_sim: board {board[0]}:{board[1]}, my cmd port {my_cmd_port}, "
          f"my scan port {args.scan_port}")

    session = int(time.time()) & 0xFFFFFFFF
    capture = open(args.capture, "a") if args.capture else None

    def send(data, port=None):
        cmd.sendto(data, (board[0], port or board[1]))
        if capture:
            capture.write(f"{time.time():.3f} TX {data.hex()}\n")
            capture.flush()

    def helo():
        flags = HELO_TAKEOVER if args.takeover else 0
        pkt = struct.pack("<4sHHHHI", b"HELO", PROTO, args.scan_port,
                          args.telem_port, flags, session)
        send(pkt)
        print(f"HELO sent (proto {PROTO}, takeover={bool(flags)}, "
              f"session {session:08x})")

    helo()

    driver = Driver()
    seq = 0
    phone_t0 = time.time()
    ping_seq = 0
    ping_sent = None
    last_ping_req = 0.0
    last_cmd_send = 0.0
    last_png = 0.0
    last_status = 0.0
    last_scan_status = 0.0

    # Raw keyboard: no curses dependency (bench machines are minimal).
    fd = sys.stdin.fileno()
    old_attrs = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        print("keys: w/s fwd/back  a/d turn  x stop  e/c estop  p ping  "
              "h re-helo  q quit")
        while True:
            now = time.time()
            phone_ms = int((now - phone_t0) * 1000)

            # --- keyboard (non-blocking) ---
            if select.select([fd], [], [], 0)[0]:
                ch = sys.stdin.read(1)
                if ch in ("w", "W"):
                    driver.forward = 1
                elif ch in ("s", "S"):
                    driver.forward = -1
                elif ch in ("a", "A"):
                    driver.turn = 1
                elif ch in ("d", "D"):
                    driver.turn = -1
                elif ch in ("x", " ", "X"):
                    driver.forward = driver.turn = 0
                elif ch in ("e", "E"):
                    driver.estop = True
                    driver.forward = driver.turn = 0
                    print(">> E-STOP latched")
                elif ch in ("c", "C"):
                    driver.estop = False
                    print(">> E-STOP cleared")
                elif ch in ("p", "P"):
                    ping_seq += 1
                    send(struct.pack("<4sII", b"PNG1", ping_seq, phone_ms))
                    ping_sent = (ping_seq, now)
                    last_ping_req = now
                    print(f"PNG1 #{ping_seq} sent")
                elif ch in ("h", "H"):
                    helo()
                elif ch in ("q", "Q"):
                    print("bye (lease expires on the board in 3 s)")
                    return

            # --- HBC2 at ~20 Hz while driving (or always, to hold the lease) ---
            if not args.no_drive and now - last_cmd_send >= 0.05:
                last_cmd_send = now
                seq += 1
                send(driver.hbc2(seq, phone_ms), args.cmd_port)

            # --- PNG1 keepalive >= 2 Hz (lease + clock sync) ---
            if now - last_png >= 0.5:
                last_png = now
                ping_seq += 1
                send(struct.pack("<4sII", b"PNG1", ping_seq, phone_ms))
                if ping_sent is None:
                    ping_sent = (ping_seq, now)

            # --- receive: HBT2/PNG2/BYE1 on the cmd socket ---
            try:
                while True:
                    d, src = cmd.recvfrom(4096)
                    if capture:
                        capture.write(f"{time.time():.3f} RX {d.hex()}\n")
                        capture.flush()
                    if d[:4] == b"HBT2":
                        line = describe_hbt2(d)
                        if line and now - last_status >= 0.5:
                            last_status = now
                            print(line)
                    elif d[:4] == b"PNG2":
                        if ping_sent and u32(d, 4) == ping_sent[0]:
                            rtt = (now - ping_sent[1]) * 1000.0
                            offset = u32(d, 12) - (ping_sent[1] - phone_t0) * 1000.0
                            print(f"PNG2 #{u32(d, 4)}  RTT {rtt:6.1f} ms  "
                                  f"esp-phone offset {offset:+.0f} ms")
                            ping_sent = None
                    elif d[:4] == b"HACK":
                        print(describe_hack(d) or d[:4])
                    elif d[:4] == b"BYE1":
                        print(describe_bye(d) or d[:4])
            except (BlockingIOError, socket.timeout):
                pass

            # --- receive: LDS2 on the scan socket ---
            try:
                while True:
                    d, src = scan.recvfrom(4096)
                    if capture:
                        capture.write(f"{time.time():.3f} RX-SCAN {d.hex()}\n")
                        capture.flush()
                    if now - last_scan_status >= 1.0:
                        last_scan_status = now
                        print(describe_lds2(d) or d[:4])
            except BlockingIOError:
                pass

            # Pace at ~50 Hz so the keyboard and sockets stay responsive.
            time.sleep(0.02)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_attrs)
        if capture:
            capture.close()
        cmd.close()
        scan.close()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nbye")
