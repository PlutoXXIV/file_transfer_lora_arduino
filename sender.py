#!/usr/bin/env python3
"""Send a file over LoRa through a node running LoRaFileTransfer.ino.

Usage:
    python sender.py --port COM5 myfile.txt
    python sender.py --port /dev/ttyACM0 photo.jpg

Works for any file type (binary-safe). Every chunk is acknowledged over the
air, and the receiving node verifies size + hash before reporting success.
"""
import argparse
import pathlib
import random
import re
import sys
import time

import serial

CHUNK = 64            # bytes per radio packet; must be <= FILE_CHUNK in the firmware
CHUNK_RETRIES = 3     # PC-level retries of a chunk after the node reports ERR
NAME_MAX = 32         # filename length limit in the firmware
DEFAULT_MAX_KB = 64


class NodeError(RuntimeError):
    pass


def fnv1a(data: bytes) -> int:
    h = 2166136261
    for b in data:
        h = ((h ^ b) * 16777619) & 0xFFFFFFFF
    return h


def clean_name(name: str) -> str:
    """ASCII-only, no path, no odd characters, at most NAME_MAX chars (keeps extension)."""
    name = re.sub(r"[^A-Za-z0-9._ -]", "_", pathlib.Path(name).name).strip() or "file.bin"
    if len(name) > NAME_MAX:
        stem, dot, ext = name.rpartition(".")
        if dot and len(ext) <= 8:
            name = stem[: NAME_MAX - len(ext) - 1] + "." + ext
        else:
            name = name[:NAME_MAX]
    return name


class Node:
    def __init__(self, ser, verbose=False):
        self.ser = ser
        self.verbose = verbose
        self.retries = 0

    def send(self, text: str):
        self.ser.write((text + "\n").encode("ascii"))

    def wait(self, match, timeout):
        """Read lines until match(tokens) is true. ERR lines raise NodeError."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            line = self.ser.readline().decode("ascii", errors="replace").strip()
            if not line:
                continue
            tok = line.split()
            if tok[0] == "ERR":
                raise NodeError(line)
            if tok[0] == "RETRY":
                self.retries += 1
            if match(tok):
                return tok
            if self.verbose:
                print("  [node]", line)
        raise TimeoutError("no response from the node")


def fmt_time(s: float) -> str:
    s = int(s)
    return f"{s // 60}m{s % 60:02d}s" if s >= 60 else f"{s}s"


def connect(node: Node, link_wait: float, check_link: bool):
    # Opening the port resets most Arduinos; ping until the sketch answers.
    time.sleep(1.5)
    node.ser.reset_input_buffer()
    tok = None
    for _ in range(10):
        node.send("PING")
        try:
            tok = node.wait(lambda t: t[0] == "PONG", 1.5)
            break
        except TimeoutError:
            continue
    if tok is None:
        raise SystemExit("The node is not answering. Check the port and that LoRaFileTransfer.ino is flashed.")
    if not check_link:
        return
    deadline = time.time() + link_wait
    warned = False
    while tok[1] != "UP":
        if time.time() > deadline:
            raise SystemExit("No link to the other node. Is it powered, on the same settings, and in range?\n"
                             "(Use --no-link-check to try anyway.)")
        if not warned:
            print("Waiting for the other node (it announces itself every few seconds)...")
            warned = True
        time.sleep(1.5)
        node.send("PING")
        try:
            tok = node.wait(lambda t: t[0] == "PONG", 2)
        except TimeoutError:
            pass
    print(f"Link up (RSSI {tok[2]} dBm, SNR {tok[3]} dB)")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", required=True, help="serial port of the sending node, e.g. COM5 or /dev/ttyACM0")
    ap.add_argument("file", help="file to send")
    ap.add_argument("--max-kb", type=int, default=DEFAULT_MAX_KB, help=f"refuse files larger than this (default {DEFAULT_MAX_KB})")
    ap.add_argument("--link-wait", type=float, default=30, help="seconds to wait for the other node (default 30)")
    ap.add_argument("--no-link-check", action="store_true", help="do not wait for the link before sending")
    ap.add_argument("-v", "--verbose", action="store_true", help="show every line from the node")
    args = ap.parse_args()

    path = pathlib.Path(args.file)
    if not path.is_file():
        raise SystemExit(f"File not found: {path}")
    data = path.read_bytes()
    if not data:
        raise SystemExit("File is empty.")
    if len(data) > args.max_kb * 1024:
        raise SystemExit(f"File is {len(data)} bytes; limit is {args.max_kb} KB (raise it with --max-kb). "
                         "LoRa is slow: roughly 100 bytes/second at SF9.")
    name = clean_name(path.name)
    digest = fnv1a(data)
    tid = random.randint(1, 255)
    chunks = [data[i:i + CHUNK] for i in range(0, len(data), CHUNK)]
    if len(chunks) > 0xFFFD:
        raise SystemExit("File too large for the protocol.")

    print(f"Sending {name}: {len(data)} bytes in {len(chunks)} chunks (~{fmt_time(len(chunks) * 0.7)} at SF9)")

    with serial.Serial(args.port, 115200, timeout=0.25, write_timeout=3) as ser:
        node = Node(ser, args.verbose)
        try:
            connect(node, args.link_wait, not args.no_link_check)

            node.send(f"BEGIN {tid} {len(data)} {digest:08X} {name}")
            node.wait(lambda t: t[0] == "READY", 45)

            started = time.time()
            sent = 0
            for seq, chunk in enumerate(chunks):
                for attempt in range(1, CHUNK_RETRIES + 1):
                    node.send(f"CHUNK {seq} {chunk.hex().upper()}")
                    try:
                        node.wait(lambda t: t[0] == "ACK" and len(t) > 1 and t[1] == str(seq), 45)
                        break
                    except NodeError as e:
                        print(f"\n  chunk {seq}: {e} (attempt {attempt}/{CHUNK_RETRIES})")
                        time.sleep(1)
                else:
                    raise SystemExit(f"\nGiving up at chunk {seq}. The link is too weak or lost.")
                sent += len(chunk)
                el = time.time() - started
                rate = sent / el if el > 0 else 0
                eta = (len(data) - sent) / rate if rate else 0
                print(f"\r  {sent * 100 // len(data):3d}%  {sent}/{len(data)} bytes  "
                      f"{rate:5.0f} B/s  ETA {fmt_time(eta)}   ", end="", flush=True)
            print()

            node.send(f"END {tid} {digest:08X}")
            tok = node.wait(lambda t: t[0] == "DONE" and len(t) > 2 and t[1] == str(tid), 45)
            elapsed = time.time() - started
            if tok[2] == "OK":
                print(f"SUCCESS: receiver verified {len(data)} bytes in {elapsed:.1f}s "
                      f"({len(data) / elapsed:.0f} B/s, {node.retries} radio retries).")
            else:
                print("FAILED: the receiver reports the data does not match. Try again.")
                sys.exit(2)
        except NodeError as e:
            raise SystemExit(f"Node error: {e}")
        except TimeoutError as e:
            raise SystemExit(f"Timed out: {e}")
        except KeyboardInterrupt:
            try:
                node.send("ABORT")
            except Exception:
                pass
            raise SystemExit("\nAborted.")


if __name__ == "__main__":
    main()
