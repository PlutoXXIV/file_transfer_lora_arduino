/*
  LoRa Messenger + File Transfer
  Arduino + Dragino-style SX1276 shield

  Built on the working messenger firmware: same radio setup, watchdog, listen-
  before-talk, jittered retries and diagnostics. On top of that it adds a
  stop-and-wait file transfer that is driven by sender.py / receiver.py.

  - Flash the SAME sketch on both boards, with NODE_A defined on one only
    (PlatformIO does this through the *_a / *_b environments).
  - Typing text in the Serial Monitor still works exactly like the messenger.
  - sender.py / receiver.py talk to the board with these serial lines
    (any board can send or receive):

      PC -> board                          board -> PC
      PING                                 PONG <UP|DOWN> <rssi> <snr>
      BEGIN <tid> <size> <hash8> <name>    READY <tid>
      CHUNK <seq> <HEX data>               ACK <seq>
      END <tid> <hash8>                    DONE <tid> <OK|FAIL>
      ABORT                                ABORTED
      (any failure)                        ERR <reason>

      receiving board -> PC:   RXBEGIN <tid> <size> <hash8> <name>
                               RXCHUNK <tid> <seq> <HEX data>
                               RXDONE  <tid> <OK|BAD|TIMEOUT|ABORTED>

  Note: a chat message that starts with "BEGIN ", "CHUNK ", "END " or "PING"
  is treated as a command, not text.
*/

#include <SPI.h>
#include <LoRa.h>

// ===== Node role: choose ONE per board =====
// PlatformIO sets this for you (envs *_a and *_b in platformio.ini).
// Arduino IDE users: uncomment the next line on board A only.
// #define NODE_A

#ifdef NODE_A
  #define MY_ADDR   0xAA
  #define PEER_ADDR 0xBB
#else
  #define MY_ADDR   0xBB
  #define PEER_ADDR 0xAA
#endif

// ===== Pins (Dragino-style shield) =====
#define PIN_SS   10
#define PIN_RST  9
#define PIN_DIO0 2

// ===== Radio settings (must match on both boards) =====
#define LORA_FREQ   868E6
#define LORA_SF     9         // 7 = ~4x faster file transfer, shorter range
#define LORA_BW     125E3
#define LORA_SYNC   0x12
#define TX_POWER    5         // start low; raise with /pow N once stable (2..17)

// Top byte of the frequency register: 0xD9 = 868 MHz, 0xD8 = 866 MHz (change with LORA_FREQ)
#define FRF_MSB     0xD9

// ===== Protocol =====
#define HDR_SIZE       5
#define TYPE_DATA      0x01
#define TYPE_ACK       0x02
#define TYPE_HELLO     0x03
#define TYPE_FBEGIN    0x10   // [tid][size u32][hash u32][name...]
#define TYPE_FDATA     0x11   // [tid][seq u16][data...]
#define TYPE_FEND      0x12   // [tid]
#define TYPE_FACK      0x13   // [tid][seq u16][status]
#define MAX_PAYLOAD    100
#define MAX_RETRIES    5
#define ACK_TIMEOUT_MS 1000UL     // plus 0-400 ms random jitter
#define DEDUPE_MS      6000UL
#define LINK_TIMEOUT   15000UL    // no frame for this long = link down
#define RESET_AFTER    20000UL    // no frame for this long = re-init radio
#define TX_TIMEOUT_MS  1500UL

// ===== File transfer =====
#define FILE_CHUNK     64         // data bytes per radio packet (sender.py must use <= this)
#define FILE_NAME_MAX  32
#define FILE_RETRIES   6
#define SEQ_BEGIN      0xFFFF     // pseudo sequence numbers used in FACK
#define SEQ_END        0xFFFE
#define RX_IDLE_ABORT  30000UL    // receiver gives up if the sender goes quiet
#define TX_IDLE_ABORT  60000UL    // sender session expires if the PC goes quiet
#define SERIAL_LINE_MAX       160        // longest accepted serial line

// ===== SX127x registers =====
#define REG_FIFO          0x00
#define REG_OPMODE        0x01
#define REG_FRF_MSB       0x06
#define REG_FIFO_PTR      0x0D
#define REG_RX_CUR_ADDR   0x10
#define REG_IRQ           0x12
#define REG_RX_BYTES      0x13
#define REG_MODEM_STAT    0x18
#define REG_PKT_SNR       0x19
#define REG_PKT_RSSI      0x1A
#define REG_RSSI          0x1B
#define REG_MODEM_CFG2    0x1E
#define REG_VERSION       0x42

// ---------- raw radio helpers ----------
uint8_t rd(uint8_t a) {
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(PIN_SS, LOW); SPI.transfer(a & 0x7F);
  uint8_t v = SPI.transfer(0);
  digitalWrite(PIN_SS, HIGH); SPI.endTransaction();
  return v;
}
void wr(uint8_t a, uint8_t v) {
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(PIN_SS, LOW); SPI.transfer(a | 0x80); SPI.transfer(v);
  digitalWrite(PIN_SS, HIGH); SPI.endTransaction();
}

// ---------- small helpers ----------
void put32(uint8_t *d, uint32_t v) {
  for (uint8_t i = 0; i < 4; i++) d[i] = (uint8_t)(v >> (8 * i));
}
uint32_t get32(const uint8_t *s) {
  uint32_t v = 0;
  for (uint8_t i = 0; i < 4; i++) v |= (uint32_t)s[i] << (8 * i);
  return v;
}
uint32_t fnvStep(uint32_t h, uint8_t b) { return (h ^ b) * 16777619UL; }
#define FNV_INIT 2166136261UL

void printHex32(uint32_t v) {
  for (int8_t s = 28; s >= 0; s -= 4) Serial.print((uint8_t)((v >> s) & 0xF), HEX);
}
int hexVal(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

// ---------- state ----------
int   lastRssi = 0;
float lastSnr  = 0;
int   txPower  = TX_POWER;

char lineBuf[SERIAL_LINE_MAX + 1];
uint8_t lineLen = 0;
bool lineOverflow = false;
uint8_t msgCounter = 0;
int16_t lastAckedId = -1;
int16_t lastRxId = -1;
unsigned long lastRxTime = 0;
unsigned long lastHeard = 0;
unsigned long lastReinit = 0;
unsigned long lastWarn = 0;
unsigned long nextBeat = 0;
unsigned long nextWatchdog = 0;
uint8_t beatSkips = 0;
bool linkUp = false;

// file transfer: sending side
bool txActive = false;
uint8_t txTid = 0;
unsigned long txLast = 0;
uint8_t txBuf[MAX_PAYLOAD];
bool fAckGot = false;
uint8_t fAckTid = 0, fAckStatus = 0;
uint16_t fAckSeq = 0;

// file transfer: receiving side
bool rxActive = false;
uint8_t rxTid = 0;
uint32_t rxSize = 0, rxHash = 0, rxGot = 0, rxRun = FNV_INIT;
uint16_t rxNext = 0;
unsigned long rxLast = 0;
bool haveDone = false;
uint8_t doneTid = 0, doneStatus = 0;

// diagnostics counters
unsigned long rxCount = 0, txCount = 0, helloCount = 0, crcErr = 0;
unsigned long cfgLost = 0, modeFix = 0, txTimeouts = 0, resetCount = 0;
unsigned long filesSent = 0, filesRecv = 0;

// ---------- radio control ----------
void rxArm() {
  LoRa.receive();
  wr(REG_IRQ, 0xFF);
}

// Registers that a power glitch / SPI fault would wipe
bool radioConfigOk() {
  if (rd(REG_VERSION) != 0x12) return false;                 // chip not answering
  if (!(rd(REG_OPMODE) & 0x80)) return false;                // fell out of LoRa mode
  if (rd(REG_FRF_MSB) != FRF_MSB) return false;              // frequency reset to default
  if ((rd(REG_MODEM_CFG2) >> 4) != LORA_SF) return false;    // SF reset to default
  return true;
}

bool initRadio() {
  LoRa.setPins(PIN_SS, PIN_RST, PIN_DIO0);
  LoRa.setSPIFrequency(1000000);
  if (!LoRa.begin(LORA_FREQ)) return false;
  LoRa.setSpreadingFactor(LORA_SF);
  LoRa.setSignalBandwidth(LORA_BW);
  LoRa.setCodingRate4(5);
  LoRa.setSyncWord(LORA_SYNC);
  LoRa.enableCrc();
  LoRa.setGain(0);                 // automatic gain control
  LoRa.setTxPower(txPower);
  rxArm();
  return radioConfigOk();
}

void recoverRadio(const __FlashStringHelper *why) {
  Serial.println(why);
  lastReinit = millis();
  if (!initRadio()) Serial.println(F("[INIT FAILED] retrying"));
  nextBeat = millis() + random(500, 2000);
}

// ---------- sending ----------
// Channel busy = signal detected / synchronised / header valid
bool channelBusy() { return rd(REG_MODEM_STAT) & 0x0B; }

void waitClear() {
  unsigned long t = millis();
  while (channelBusy() && millis() - t < 500) delay(random(5, 30));
}

bool sendPacket(uint8_t type, uint8_t id, const uint8_t *p, uint8_t len) {
  waitClear();
  if (!LoRa.beginPacket()) { rxArm(); return false; }
  LoRa.write(PEER_ADDR); LoRa.write(MY_ADDR); LoRa.write(id);
  LoRa.write(type);      LoRa.write(len);
  if (len) LoRa.write(p, len);
  LoRa.endPacket(true);                         // async: we poll TX-done ourselves

  unsigned long t = millis();
  while (!(rd(REG_IRQ) & 0x08)) {               // TxDone
    if (millis() - t > TX_TIMEOUT_MS) {
      txTimeouts++;
      recoverRadio(F("[TX TIMEOUT] radio never finished sending - power/SPI fault"));
      return false;
    }
  }
  txCount++;
  rxArm();
  return true;
}

void sendAck(uint8_t id) {
  delay(30);                                    // let the peer return to RX
  sendPacket(TYPE_ACK, id, NULL, 0);
}

void sendFileAck(uint8_t tid, uint16_t seq, uint8_t status) {
  uint8_t p[4] = { tid, (uint8_t)(seq & 0xFF), (uint8_t)(seq >> 8), status };
  delay(30);
  sendPacket(TYPE_FACK, (uint8_t)seq, p, 4);
}

// ---------- receiving ----------
int rawRead(uint8_t *buf, int maxLen) {
  uint8_t flags = rd(REG_IRQ);
  if (!(flags & 0x40)) return 0;                // no RxDone
  if (flags & 0x20) { crcErr++; wr(REG_IRQ, 0xFF); return 0; }   // drop CRC errors
  lastRssi = (int)rd(REG_PKT_RSSI) - 157;
  lastSnr  = (int8_t)rd(REG_PKT_SNR) / 4.0;
  uint8_t n = rd(REG_RX_BYTES);
  wr(REG_FIFO_PTR, rd(REG_RX_CUR_ADDR));
  int got = 0;
  for (uint8_t i = 0; i < n; i++) {
    uint8_t c = rd(REG_FIFO);
    if (got < maxLen) buf[got++] = c;
  }
  wr(REG_IRQ, 0xFF);
  return got;
}

void markHeard() {
  lastHeard = millis();
  if (lastRssi > -35 && millis() - lastWarn > 5000) {
    lastWarn = millis();
    Serial.println(F("[WARN] signal very strong - move boards apart"));
  }
  if (!linkUp) {
    linkUp = true;
    Serial.print(F("[LINK UP] RSSI "));
    Serial.print(lastRssi);
    Serial.print(F(" dBm, SNR "));
    Serial.print(lastSnr, 1);
    Serial.println(F(" dB"));
  }
}

// ---------- file receive handlers ----------
void onFileBegin(const uint8_t *p, uint8_t len) {
  uint8_t tid = p[0];
  if (!(rxActive && rxTid == tid)) {            // a repeated BEGIN (lost ACK) is only re-ACKed
    if (rxActive) {
      Serial.print(F("RXDONE ")); Serial.print(rxTid); Serial.println(F(" ABORTED"));
    }
    rxActive = true;
    rxTid = tid;
    rxSize = get32(p + 1);
    rxHash = get32(p + 5);
    rxGot = 0; rxNext = 0; rxRun = FNV_INIT;
    Serial.print(F("RXBEGIN "));
    Serial.print(tid); Serial.print(' ');
    Serial.print(rxSize); Serial.print(' ');
    printHex32(rxHash); Serial.print(' ');
    for (uint8_t i = 9; i < len; i++) {
      char c = (char)p[i];
      Serial.write((c < 33 && c != ' ') || c > 126 ? '_' : c);
    }
    Serial.println();
  }
  rxLast = millis();
  sendFileAck(tid, SEQ_BEGIN, 0);
}

void onFileData(const uint8_t *p, uint8_t len) {
  uint8_t tid = p[0];
  uint16_t seq = p[1] | ((uint16_t)p[2] << 8);
  if (!rxActive || tid != rxTid) return;
  rxLast = millis();
  if (seq == rxNext) {                          // the chunk we are waiting for
    uint8_t n = len - 3;
    Serial.print(F("RXCHUNK "));
    Serial.print(tid); Serial.print(' ');
    Serial.print(seq); Serial.print(' ');
    for (uint8_t i = 0; i < n; i++) {
      uint8_t c = p[3 + i];
      rxRun = fnvStep(rxRun, c);
      if (c < 16) Serial.print('0');
      Serial.print(c, HEX);
    }
    Serial.println();
    rxGot += n;
    rxNext++;
    sendFileAck(tid, seq, 0);
  } else if (seq < rxNext) {                    // duplicate: our ACK was lost, ACK again
    sendFileAck(tid, seq, 0);
  }                                             // seq > rxNext: ignore, sender will retry
}

void onFileEnd(const uint8_t *p) {
  uint8_t tid = p[0];
  if (rxActive && tid == rxTid) {
    bool ok = (rxGot == rxSize) && (rxRun == rxHash);
    doneTid = tid;
    doneStatus = ok ? 0 : 1;
    haveDone = true;
    rxActive = false;
    if (ok) filesRecv++;
    Serial.print(F("RXDONE ")); Serial.print(tid);
    Serial.println(ok ? F(" OK") : F(" BAD"));
    sendFileAck(tid, SEQ_END, doneStatus);
  } else if (haveDone && tid == doneTid) {      // duplicate END: repeat the verdict
    sendFileAck(tid, SEQ_END, doneStatus);
  }
}

void pollRadio() {
  uint8_t b[HDR_SIZE + MAX_PAYLOAD];
  int n = rawRead(b, sizeof(b));
  if (n < HDR_SIZE) return;

  uint8_t dest = b[0], src = b[1], id = b[2], type = b[3], len = b[4];
  if (dest != MY_ADDR || src != PEER_ADDR || len != n - HDR_SIZE) return;

  rxCount++;
  markHeard();
  const uint8_t *p = b + HDR_SIZE;

  if (type == TYPE_ACK && len == 0) {
    lastAckedId = id;
  } else if (type == TYPE_HELLO) {
    helloCount++;
  } else if (type == TYPE_DATA) {
    bool dup = (id == lastRxId) && (millis() - lastRxTime < DEDUPE_MS);
    if (!dup) {
      lastRxId = id;
      lastRxTime = millis();
      Serial.print(F("[RX] "));
      for (int i = 0; i < len; i++) Serial.write(p[i]);
      Serial.print(F("   ("));
      Serial.print(lastRssi);
      Serial.print(F(" dBm, SNR "));
      Serial.print(lastSnr, 1);
      Serial.println(F(")"));
    }
    sendAck(id);                                // always ACK, even duplicates
  } else if (type == TYPE_FBEGIN && len >= 10) {
    onFileBegin(p, len);
  } else if (type == TYPE_FDATA && len >= 4) {
    onFileData(p, len);
  } else if (type == TYPE_FEND && len == 1) {
    onFileEnd(p);
  } else if (type == TYPE_FACK && len == 4) {
    fAckTid = p[0];
    fAckSeq = p[1] | ((uint16_t)p[2] << 8);
    fAckStatus = p[3];
    fAckGot = true;
  }
}

// ---------- background tasks ----------
// Watchdog + link state. Safe to call anywhere (never transmits).
void watchdog() {
  unsigned long now = millis();
  if (now < nextWatchdog) return;
  nextWatchdog = now + 500;

  if (!radioConfigOk()) {
    cfgLost++;
    recoverRadio(F("[CFG LOST] radio registers wrong - power glitch or SPI fault"));
    return;
  }
  if ((rd(REG_OPMODE) & 0x07) != 0x05) {        // not in continuous RX
    modeFix++;
    rxArm();
  }
  if (linkUp && now - lastHeard > LINK_TIMEOUT) {
    linkUp = false;
    Serial.println(F("[LINK DOWN] nothing heard from peer"));
  }
  if (now - lastHeard > RESET_AFTER && now - lastReinit > RESET_AFTER) {
    resetCount++;
    recoverRadio(F("[RADIO RESET] silent too long, re-initialising"));
  }
  if (rxActive && now - rxLast > RX_IDLE_ABORT) {
    Serial.print(F("RXDONE ")); Serial.print(rxTid); Serial.println(F(" TIMEOUT"));
    rxActive = false;
  }
  if (txActive && now - txLast > TX_IDLE_ABORT) txActive = false;
}

// Heartbeat: skip this slot if the channel is busy or a frame is waiting.
// Stays silent during a file transfer so it never collides with data/ACKs.
void heartbeat() {
  unsigned long now = millis();
  if (now < nextBeat) return;
  if (txActive || rxActive) {
    nextBeat = now + random(3000, 5000);
    return;
  }
  if ((channelBusy() || (rd(REG_IRQ) & 0x40)) && beatSkips < 8) {
    beatSkips++;
    nextBeat = now + random(100, 400);
    return;
  }
  beatSkips = 0;
  sendPacket(TYPE_HELLO, 0, NULL, 0);
  nextBeat = millis() + random(3000, 5000);
}

// ---------- reliable send: chat text ----------
bool sendReliable(const char *text, uint8_t len) {
  uint8_t id = msgCounter++;
  lastAckedId = -1;
  for (uint8_t a = 1; a <= MAX_RETRIES; a++) {
    sendPacket(TYPE_DATA, id, (const uint8_t *)text, len);
    unsigned long wait = ACK_TIMEOUT_MS + random(0, 400);
    unsigned long t = millis();
    while (millis() - t < wait) {
      pollRadio();
      watchdog();                               // keep recovery running while waiting
      if (lastAckedId == id) {
        Serial.println(F("[DELIVERED]"));
        return true;
      }
    }
    Serial.print(F("[retry "));
    Serial.print(a);
    Serial.println(F("]"));
  }
  Serial.println(F("[FAILED] no ACK"));
  return false;
}

// ---------- reliable send: file packets ----------
// Payload is already in txBuf. Returns true once the peer ACKs (txTid, seq).
bool sendFileReliable(uint8_t type, uint16_t seq, uint8_t len) {
  for (uint8_t a = 1; a <= FILE_RETRIES; a++) {
    fAckGot = false;
    sendPacket(type, (uint8_t)seq, txBuf, len);
    unsigned long wait = ACK_TIMEOUT_MS + random(0, 400);
    unsigned long t = millis();
    while (millis() - t < wait) {
      pollRadio();
      watchdog();
      if (fAckGot && fAckTid == txTid && fAckSeq == seq) return true;
    }
    Serial.print(F("RETRY ")); Serial.println(a);
  }
  return false;
}

// ---------- file commands from the PC ----------
void cmdBegin(char *s) {
  char *e;
  uint32_t tid  = strtoul(s, &e, 10);
  uint32_t size = strtoul(e, &e, 10);
  uint32_t hash = strtoul(e, &e, 16);
  while (*e == ' ') e++;
  if (tid > 255 || *e == '\0') { Serial.println(F("ERR bad BEGIN")); return; }
  uint8_t nlen = strlen(e);
  if (nlen > FILE_NAME_MAX) nlen = FILE_NAME_MAX;

  txTid = (uint8_t)tid;
  txActive = true;
  txLast = millis();
  txBuf[0] = txTid;
  put32(txBuf + 1, size);
  put32(txBuf + 5, hash);
  memcpy(txBuf + 9, e, nlen);
  if (sendFileReliable(TYPE_FBEGIN, SEQ_BEGIN, 9 + nlen)) {
    Serial.print(F("READY ")); Serial.println(txTid);
  } else {
    txActive = false;
    Serial.println(F("ERR begin no-ack"));
  }
}

void cmdChunk(char *s) {
  if (!txActive) { Serial.println(F("ERR no-session")); return; }
  char *e;
  uint32_t seq = strtoul(s, &e, 10);
  while (*e == ' ') e++;
  uint8_t hl = strlen(e);
  if (seq > 0xFFFD || hl == 0 || (hl & 1) || hl / 2 > FILE_CHUNK) {
    Serial.println(F("ERR bad-chunk")); return;
  }
  uint8_t n = hl / 2;
  txBuf[0] = txTid;
  txBuf[1] = (uint8_t)(seq & 0xFF);
  txBuf[2] = (uint8_t)(seq >> 8);
  for (uint8_t i = 0; i < n; i++) {
    int hi = hexVal(e[2 * i]), lo = hexVal(e[2 * i + 1]);
    if (hi < 0 || lo < 0) { Serial.println(F("ERR bad-hex")); return; }
    txBuf[3 + i] = (uint8_t)((hi << 4) | lo);
  }
  txLast = millis();
  if (sendFileReliable(TYPE_FDATA, (uint16_t)seq, 3 + n)) {
    Serial.print(F("ACK ")); Serial.println((uint16_t)seq);
  } else {
    Serial.print(F("ERR chunk ")); Serial.print((uint16_t)seq); Serial.println(F(" no-ack"));
  }
}

void cmdEnd() {
  if (!txActive) { Serial.println(F("ERR no-session")); return; }
  txBuf[0] = txTid;
  txLast = millis();
  if (sendFileReliable(TYPE_FEND, SEQ_END, 1)) {
    Serial.print(F("DONE ")); Serial.print(txTid);
    Serial.println(fAckStatus == 0 ? F(" OK") : F(" FAIL"));
    if (fAckStatus == 0) filesSent++;
    txActive = false;
  } else {
    Serial.println(F("ERR end no-ack"));
  }
}

// ---------- serial input ----------
void handleLine(char *s, uint8_t len) {
  if (strcmp(s, "/status") == 0) {
    Serial.print(F("Node 0x")); Serial.print(MY_ADDR, HEX);
    Serial.print(F(" | link ")); Serial.print(linkUp ? F("UP") : F("DOWN"));
    Serial.print(F(" | RSSI ")); Serial.print(lastRssi);
    Serial.print(F(" dBm, SNR ")); Serial.print(lastSnr, 1);
    Serial.print(F(" dB | SF")); Serial.print(LORA_SF);
    Serial.print(F(" | pwr ")); Serial.println(txPower);
    Serial.print(F("  rx ")); Serial.print(rxCount);
    Serial.print(F(" tx ")); Serial.print(txCount);
    Serial.print(F(" hellos ")); Serial.print(helloCount);
    Serial.print(F(" crcErr ")); Serial.print(crcErr);
    Serial.print(F(" | cfgLost ")); Serial.print(cfgLost);
    Serial.print(F(" txTimeout ")); Serial.print(txTimeouts);
    Serial.print(F(" modeFix ")); Serial.print(modeFix);
    Serial.print(F(" resets ")); Serial.println(resetCount);
    Serial.print(F("  files sent ")); Serial.print(filesSent);
    Serial.print(F(" received ")); Serial.println(filesRecv);
    return;
  }
  if (strcmp(s, "/dump") == 0) {
    Serial.print(F("ver 0x"));  Serial.print(rd(REG_VERSION), HEX);
    Serial.print(F(" op 0x"));  Serial.print(rd(REG_OPMODE), HEX);
    Serial.print(F(" frf 0x")); Serial.print(rd(REG_FRF_MSB), HEX);
    Serial.print(F(" cfg2 0x")); Serial.print(rd(REG_MODEM_CFG2), HEX);
    Serial.print(F(" stat 0x")); Serial.print(rd(REG_MODEM_STAT), HEX);
    Serial.print(F(" irq 0x")); Serial.println(rd(REG_IRQ), HEX);
    return;
  }
  if (strcmp(s, "/rssi") == 0) {                // peak RF energy over 4 s
    int mx = -200; unsigned long t = millis();
    while (millis() - t < 4000) {
      int r = (int)rd(REG_RSSI) - 157;
      if (r > mx) mx = r;
    }
    Serial.print(F("peak RSSI 4s: ")); Serial.println(mx);
    return;
  }
  if (strncmp(s, "/pow ", 5) == 0) {
    int p = atoi(s + 5);
    if (p >= 2 && p <= 17) {
      txPower = p;
      LoRa.setTxPower(txPower);
      Serial.print(F("TX power ")); Serial.println(txPower);
    } else {
      Serial.println(F("[ERR] /pow 2..17"));
    }
    return;
  }

  // ---- machine commands used by sender.py ----
  if (strcmp(s, "PING") == 0) {
    Serial.print(F("PONG ")); Serial.print(linkUp ? F("UP") : F("DOWN"));
    Serial.print(' '); Serial.print(lastRssi);
    Serial.print(' '); Serial.println(lastSnr, 1);
    return;
  }
  if (strncmp(s, "BEGIN ", 6) == 0) { cmdBegin(s + 6); return; }
  if (strncmp(s, "CHUNK ", 6) == 0) { cmdChunk(s + 6); return; }
  if (strncmp(s, "END", 3) == 0 && (s[3] == '\0' || s[3] == ' ')) { cmdEnd(); return; }
  if (strcmp(s, "ABORT") == 0) {
    txActive = false;
    Serial.println(F("ABORTED"));
    return;
  }

  // ---- plain chat message ----
  if (len > MAX_PAYLOAD) {
    Serial.println(F("[ERR] message too long (max 100)"));
    return;
  }
  Serial.print(F("[ME] "));
  Serial.println(s);
  sendReliable(s, len);
}

void readSerialInput() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      if (lineOverflow) {
        Serial.println(F("[ERR] line too long"));
      } else if (lineLen > 0) {
        lineBuf[lineLen] = '\0';
        handleLine(lineBuf, lineLen);
      }
      lineLen = 0;
      lineOverflow = false;
    } else if (lineLen < SERIAL_LINE_MAX) {
      lineBuf[lineLen++] = c;
    } else {
      lineOverflow = true;
    }
  }
}

void setup() {
  Serial.begin(115200);
  randomSeed(analogRead(A0) ^ micros());
  msgCounter = random(256);

  while (!initRadio()) {
    Serial.println(F("LoRa init failed - check shield/power, retrying"));
    delay(1000);
  }

  lastHeard = lastReinit = millis();
  nextBeat = millis() + random(500, 2000);
  Serial.print(F("Node 0x")); Serial.print(MY_ADDR, HEX);
  Serial.print(F(" ready, SF")); Serial.println(LORA_SF);
  Serial.println(F("Type a message + Enter. /status /dump /rssi /pow N"));
  Serial.println(F("File transfer: run sender.py / receiver.py on the PCs"));
}

void loop() {
  pollRadio();
  watchdog();
  heartbeat();
  readSerialInput();
}
