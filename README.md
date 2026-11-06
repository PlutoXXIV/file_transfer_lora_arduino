# LoRa File Transfer

Send any file from one laptop to another over a LoRa radio link. No internet, WiFi or mobile network needed.
Two Arduino boards with LoRa shields act as radio modems; the laptops just run one Python tool.

```
 laptop ──USB──▶ Arduino A ~~~ LoRa 868 MHz ~~~ Arduino B ──USB──▶ laptop
 lora.py send                                                      lora.py receive
```

- Every 64-byte chunk is acknowledged and resent if lost.
- The receiving board checks the file size and hash, so a successful transfer is a verified one.
- Binary-safe: text, images, PDFs, zips, anything.
- The boards also work as a text messenger (see [Chat mode](#chat-mode)).

## What you need

| Item | Notes |
|---|---|
| 2 × Arduino Uno R4 WiFi (or Uno R3) | each with a Dragino-style SX1276 LoRa shield (pins: SS 10, RST 9, DIO0 2) |
| 2 × USB data cables | charge-only cables will not work |
| Python 3.8 or newer | the only thing you install yourself |
| Internet (first run only) | downloads the PlatformIO toolchain and libraries |

No Arduino IDE needed. Everything else (PlatformIO, pyserial, the LoRa library) is installed automatically into a private `.venv` folder inside this project.

## Quick start

### If you dont have Git installed - 
- you can manually download the zip of this project from the green colored 'code'
- then extract the folder to your desired location
- open the terminal and go to that location
- then continue from stem 1. Flash the two boards

### or install git and follow the below instructions

```terminal
git clone https://github.com/PlutoXXIV/file_transfer_lora_arduino
cd lora-file-transfer
```

**1. Flash the two boards** (one at a time; the first build takes a few minutes)

```terminal
python lora.py flash --node a      # plug in board A, run, then unplug
python lora.py flash --node b      # plug in board B, run
```

**2. Receive** (on the receiving laptop, board B plugged in)

```terminal
python lora.py receive
```

**3. Send** (on the sending laptop, board A plugged in)

```terminal
python lora.py send myfile.pdf
```

The sender prints `SUCCESS: receiver verified ...` and the file appears in `received_files/` on the receiving laptop.

> Windows: use `py` instead of `python` if `python` is not recognised.
> Uno R3 boards: add `--board r3` to the flash commands.
> Both boards on one laptop? Auto-detection stops and asks you to pick with `--port`.

## Commands

| Command | What it does |
|---|---|
| `python lora.py flash --node a\|b [--board r4\|r3] [--port P]` | build and upload the firmware (`a` and `b` are the two node addresses) |
| `python lora.py receive [--out DIR] [--port P]` | wait for files, save them to `received_files/` (or `DIR`) |
| `python lora.py send FILE [--port P] [-v]` | send a file (`-v` shows every line from the board) |
| `python lora.py ports` | list serial ports |
| `python lora.py monitor` | serial monitor for chat mode |

The serial port is detected automatically; use `--port` (for example `COM5` or `/dev/cu.usbmodem1101`) only when it asks you to.

## What you will see

Sender:
```
Link up (RSSI -62 dBm, SNR 9.5 dB)
Sending photo.jpg: 3000 bytes in 47 chunks (~32s at SF9)
  100%  3000/3000 bytes  ...
SUCCESS: receiver verified 3000 bytes in ...
```
Receiver:
```
Incoming: photo.jpg (3000 bytes)
  100%  3000/3000 bytes
SUCCESS: saved received_files/photo.jpg (3000 bytes, ...)
```
While a file is arriving it is stored as `photo.jpg.part` and renamed only after size and hash match. Existing files are never overwritten (`photo_1.jpg`, `photo_2.jpg`, ...).

## Chat mode

Open `python lora.py monitor`, type a line and press Enter. The other board prints it as `[RX] ...`. Useful for checking that the link works before sending a file.

## Settings

Radio settings live at the top of `src/LoRaFileTransfer.ino` and must be identical on both boards (they already are).

| Setting | Default | Notes |
|---|---|---|
| `LORA_FREQ` / `FRF_MSB` | 868E6 / 0xD9 | change both together; 866E6 uses 0xD8 |
| `LORA_SF` | 9 | 7 is about 4× faster but shorter range |
| `TX_POWER` | 5 | raise (up to 17) only if the link is weak |
| `FILE_CHUNK` | 64 | bytes per radio packet (keep `CHUNK` in `sender.py` equal or smaller) |

Check your local rules before transmitting. For example, India's licence-free band is 865-867 MHz, so 868 MHz should be changed to 866 MHz there.

## How it works

1. `sender.py` reads the file, computes a hash, cuts it into 64-byte chunks and sends them to the board as hex text lines.
2. The board transmits each chunk and waits for an ACK from the other board (up to 6 resends).
3. The receiving board accepts chunks in order only, hands them to `receiver.py`, and verifies size + hash at the end.
4. `receiver.py` writes the data to a `.part` file and renames it when verified.

Speed is roughly 100 bytes per second at the default settings, so a 1 KB file takes about 10 seconds. The default size limit is 64 KB.

## Troubleshooting

| Problem | Fix |
|---|---|
| `No Arduino found` | use a data USB cable; run `python lora.py ports`; install the CH340 driver if your board is a clone |
| Port busy / permission denied | close Arduino IDE, serial monitors and any other running `lora.py` |
| Flashing fails on Uno R4 WiFi | double-press the reset button, then run the command again |
| `Waiting for the other node...` then gives up | other board off, not flashed with the other node letter, or out of range; start close together |
| `FAILED verification` | resend; move the boards 1-2 m apart if signal is very strong, or closer if weak |
| `[WARN] signal very strong` on the board | boards are too close; move them apart |
| `Could not create a virtual environment` (Linux) | `sudo apt install python3-venv` |

## Project layout

```
lora.py              the one tool users run (sets up .venv, flashes, sends, receives)
sender.py            file sender (also usable on its own: python sender.py --port P file)
receiver.py          file receiver (also usable on its own)
requirements.txt     pyserial + platformio
platformio.ini       boards, library (sandeepmistry/LoRa), node A/B environments
src/LoRaFileTransfer.ino   firmware (messenger + file transfer)
```

Arduino IDE users: open `src/LoRaFileTransfer.ino`, install the "LoRa" library by Sandeep Mistry, uncomment `#define NODE_A` on board A only, and use `sender.py` / `receiver.py` directly.

## Limitations

- Slow by nature (LoRa): not meant for large files.
- One file at a time, one direction per transfer.
- Data is sent unencrypted.
- `receiver.py` must be running while the file is sent: the Arduino does not store the file.

## Status

Developed and tested on Arduino Uno R4 WiFi boards with a macOS receiver. The `r3` (Uno R3) configuration is included but untested; the R3 has only 2 KB of RAM, so reduce `FILE_CHUNK` if the build reports too little memory.
