#!/usr/bin/env python3
"""LoRa file transfer - one tool for setup, flashing, sending and receiving.

  python lora.py flash --board r4 --node a    build + upload the firmware (board A)
  python lora.py flash --board r4 --node b    ... and for board B
  python lora.py send myfile.pdf              send a file
  python lora.py receive                      receive files into ./received_files
  python lora.py ports                        list connected serial ports
  python lora.py monitor                      open a serial monitor (text chat)

The first run creates a private .venv and installs pyserial + PlatformIO into it
(needs internet once). Nothing is installed system-wide.
"""
import argparse
import os
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
VENV = ROOT / ".venv"
VENV_PY = VENV / ("Scripts/python.exe" if os.name == "nt" else "bin/python")
BOARDS = {"r4": "uno_r4_wifi", "r3": "uno"}   # --board value -> env prefix in platformio.ini
USB_VIDS = {0x2341, 0x2A03, 0x1A86, 0x0403, 0x10C4}  # Arduino, Arduino.org, CH340, FTDI, CP210x


def run(cmd, **kwargs):
    print("$", " ".join(str(c) for c in cmd), flush=True)
    subprocess.run([str(c) for c in cmd], check=True, **kwargs)


def in_venv():
    return Path(sys.prefix).resolve() == VENV.resolve()


def relaunch_in_venv():
    """First run: create .venv and install requirements, then re-run this script inside it."""
    if in_venv():
        return
    if sys.version_info < (3, 8):
        sys.exit("Python 3.8 or newer is required.")
    if not VENV_PY.exists():
        print("First run: creating a private Python environment in .venv (one time)...")
        try:
            run([sys.executable, "-m", "venv", VENV])
        except subprocess.CalledProcessError:
            sys.exit("Could not create a virtual environment.\n"
                     "On Debian/Ubuntu run: sudo apt install python3-venv")
    check = subprocess.call([VENV_PY, "-c", "import serial, platformio"],
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    if check != 0:
        print("Installing pyserial and PlatformIO into .venv (one time, needs internet)...")
        try:
            run([VENV_PY, "-m", "pip", "install", "--quiet", "-r", ROOT / "requirements.txt"])
        except subprocess.CalledProcessError:
            sys.exit("pip install failed. Check your internet connection and try again.")
    sys.exit(subprocess.call([str(VENV_PY), str(Path(__file__).resolve()), *sys.argv[1:]]))


def candidate_ports():
    from serial.tools import list_ports
    found = [p for p in list_ports.comports() if p.vid in USB_VIDS]
    # macOS lists every board twice (/dev/cu.* and /dev/tty.*); keep the cu.* one.
    return [p for p in found if not p.device.startswith("/dev/tty.")]


def find_port(explicit):
    if explicit:
        return explicit
    found = candidate_ports()
    if len(found) == 1:
        print(f"Using {found[0].device} ({found[0].description})")
        return found[0].device
    if not found:
        sys.exit("No Arduino found. Plug it in (use a data USB cable), or pass --port.\n"
                 "See all ports with:  python lora.py ports")
    sys.exit("More than one board found. Choose one with --port:\n"
             + "\n".join(f"  {p.device}   {p.description}" for p in found))


def cmd_ports(args):
    from serial.tools import list_ports
    ports = list(list_ports.comports())
    if not ports:
        print("No serial ports found.")
    for p in ports:
        mark = "  <- looks like an Arduino" if p.vid in USB_VIDS else ""
        print(f"{p.device}   {p.description}{mark}")


def cmd_flash(args):
    env = f"{BOARDS[args.board]}_{args.node}"
    port = find_port(args.port)
    print(f"Building and uploading '{env}' to {port}. The first build downloads the toolchain (a few minutes).")
    try:
        run([VENV_PY, "-m", "platformio", "run", "-d", ROOT, "-e", env,
             "-t", "upload", "--upload-port", port])
    except subprocess.CalledProcessError:
        sys.exit("\nFlashing failed. Common fixes:\n"
                 "  - Close any serial monitor that is using the board.\n"
                 "  - Uno R4 WiFi: double-press the reset button, then run the command again.\n"
                 "  - Clone boards (CH340 chip) may need the CH340 USB driver on Windows/macOS.\n"
                 "  - Use a USB data cable, not a charge-only cable.")
    print(f"\nDone: board on {port} is flashed as node {args.node.upper()}.")


def cmd_send(args):
    import sender
    sys.argv = ["sender.py", "--port", find_port(args.port), args.file]
    if args.verbose:
        sys.argv.append("-v")
    sender.main()


def cmd_receive(args):
    import receiver
    sys.argv = ["receiver.py", "--port", find_port(args.port), "--out", args.out]
    receiver.main()


def cmd_monitor(args):
    run([VENV_PY, "-m", "platformio", "device", "monitor", "-p", find_port(args.port), "-b", "115200"])


def main():
    relaunch_in_venv()
    ap = argparse.ArgumentParser(description="LoRa file transfer")
    sub = ap.add_subparsers(dest="cmd", required=True)

    f = sub.add_parser("flash", help="build and upload the firmware")
    f.add_argument("--board", choices=BOARDS, default="r4", help="r4 = Uno R4 WiFi (default), r3 = Uno R3")
    f.add_argument("--node", choices=["a", "b"], required=True, help="one board must be a, the other b")
    f.add_argument("--port", help="serial port (auto-detected if omitted)")
    f.set_defaults(fn=cmd_flash)

    s = sub.add_parser("send", help="send a file")
    s.add_argument("file")
    s.add_argument("--port", help="serial port (auto-detected if omitted)")
    s.add_argument("-v", "--verbose", action="store_true", help="show every line from the board")
    s.set_defaults(fn=cmd_send)

    r = sub.add_parser("receive", help="receive files (keep running)")
    r.add_argument("--port", help="serial port (auto-detected if omitted)")
    r.add_argument("--out", default="received_files", help="output folder (default: received_files)")
    r.set_defaults(fn=cmd_receive)

    p = sub.add_parser("ports", help="list serial ports")
    p.set_defaults(fn=cmd_ports)

    m = sub.add_parser("monitor", help="serial monitor (type text to chat with the other board)")
    m.add_argument("--port", help="serial port (auto-detected if omitted)")
    m.set_defaults(fn=cmd_monitor)

    args = ap.parse_args()
    args.fn(args)


if __name__ == "__main__":
    main()
