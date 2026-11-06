#!/usr/bin/env python3
"""Receive files over LoRa from a node running LoRaFileTransfer.ino.

Usage:
    python receiver.py --port COM6
    python receiver.py --port /dev/ttyACM1 --out inbox

Leave it running; every incoming file is written to the output folder. Data goes
to "<name>.part" and is renamed only after size + hash are verified.
"""
import argparse
import pathlib
import re
import time

import serial


def fnv1a_step(h: int, data: bytes) -> int:
    for b in data:
        h = ((h ^ b) * 16777619) & 0xFFFFFFFF
    return h


def safe_name(name: str) -> str:
    name = name.replace("\\", "/").split("/")[-1].strip()
    name = re.sub(r"[^A-Za-z0-9._ -]", "_", name)
    return name if name and name not in (".", "..") else "received.bin"


def unique_path(p: pathlib.Path) -> pathlib.Path:
    if not p.exists():
        return p
    for i in range(1, 1000):
        q = p.with_name(f"{p.stem}_{i}{p.suffix}")
        if not q.exists():
            return q
    return p


class Transfer:
    def __init__(self, tid, size, expected_hash, name, outdir):
        self.tid, self.size, self.expected_hash, self.name = tid, size, expected_hash, name
        self.final = unique_path(outdir / name)
        self.part = self.final.with_name(self.final.name + ".part")
        self.f = self.part.open("wb")
        self.next_seq = 0
        self.got = 0
        self.hash = 2166136261
        self.started = time.time()

    def add(self, chunk: bytes):
        self.f.write(chunk)
        self.f.flush()
        self.got += len(chunk)
        self.hash = fnv1a_step(self.hash, chunk)
        self.next_seq += 1

    def close(self):
        if not self.f.closed:
            self.f.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", required=True, help="serial port of the receiving node, e.g. COM6 or /dev/ttyACM1")
    ap.add_argument("--out", default="received_files", help="output directory (default: received_files)")
    ap.add_argument("-v", "--verbose", action="store_true", help="also print raw RXCHUNK lines")
    args = ap.parse_args()

    outdir = pathlib.Path(args.out)
    outdir.mkdir(parents=True, exist_ok=True)
    xfer = None

    try:
        with serial.Serial(args.port, 115200, timeout=1) as ser:
            time.sleep(2)  # many Arduinos reset when the port opens
            ser.reset_input_buffer()
            print(f"Listening on {args.port}. Saving to {outdir.resolve()}. Press Ctrl+C to stop.")
            while True:
                line = ser.readline().decode("ascii", errors="replace").strip()
                if not line:
                    continue
                parts = line.split(" ", 4)
                kind = parts[0]

                try:
                    if kind == "RXBEGIN" and len(parts) >= 5:
                        if xfer:
                            xfer.close()
                            print(f"\nPrevious transfer was cut off; partial data kept at {xfer.part}")
                        xfer = Transfer(int(parts[1]), int(parts[2]), int(parts[3], 16),
                                        safe_name(parts[4]), outdir)
                        print(f"\nIncoming: {xfer.name} ({xfer.size} bytes)")

                    elif kind == "RXCHUNK" and len(parts) == 4:
                        if xfer is None or int(parts[1]) != xfer.tid:
                            continue
                        seq = int(parts[2])
                        if seq != xfer.next_seq:
                            print(f"\nWarning: expected chunk {xfer.next_seq}, got {seq}; skipped.")
                            continue
                        xfer.add(bytes.fromhex(parts[3]))
                        pct = xfer.got * 100 // xfer.size if xfer.size else 100
                        print(f"\r  {pct:3d}%  {xfer.got}/{xfer.size} bytes   ", end="", flush=True)
                        if args.verbose:
                            print("\n[node]", line)

                    elif kind == "RXDONE" and len(parts) >= 3:
                        if xfer is None or int(parts[1]) != xfer.tid:
                            continue
                        radio_status = parts[2]
                        xfer.close()
                        content = xfer.part.read_bytes()
                        h = fnv1a_step(2166136261, content)
                        good = (radio_status == "OK" and len(content) == xfer.size
                                and h == xfer.expected_hash)
                        print()
                        if good:
                            xfer.part.replace(xfer.final)
                            print(f"SUCCESS: saved {xfer.final} ({len(content)} bytes, "
                                  f"{time.time() - xfer.started:.1f}s)")
                        else:
                            print(f"FAILED verification: size {len(content)}/{xfer.size}, "
                                  f"hash {h:08X}/{xfer.expected_hash:08X}, radio says {radio_status}")
                            print(f"Partial data kept at {xfer.part}")
                        xfer = None

                    else:
                        print("[node]", line)  # chat messages, link status, warnings
                except ValueError:
                    print("\nWarning: malformed line ignored:", line[:60])
    except serial.SerialException as e:
        raise SystemExit(f"\nSerial error: {e}")
    except KeyboardInterrupt:
        if xfer:
            xfer.close()
            print(f"\nStopped mid-transfer; partial data kept at {xfer.part}")
        else:
            print("\nStopped.")


if __name__ == "__main__":
    main()
