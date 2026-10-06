// ===========================================================================================
// serial_test  VERSION 1.0   (2026-10-06)      <-- if you do not see this line, the IDE has an older copy
//
// Purpose: find out why Serial output from the UNO R4 WiFi never reaches the PC.
// It touches nothing but USB serial, the built-in LED and the 12x8 LED matrix (WiFi board only).
// Every 8 seconds it switches to the next way of sending text, 5 stages, A to E, then repeats:
//   A  Serial.println("...")                      the normal way
//   B  Serial.write(buffer, length)               the raw write
//   C  Serial.dtr() once, then println            forces the core to treat the port as connected (ignores DTR)
//   D  println followed by Serial.flush()
//   E  one byte at a time with Serial.write(char)
// Open the Serial Monitor (or ask Claude to listen) and note WHICH stage letters produce text.
//
// Matrix:   left glyph  = the current stage letter (A-E)
//           right glyph = the number of bytes the core said it accepted for the last send (hex, 0 = nothing taken)
//           bottom row (1 = leftmost):  1 lit = Serial says a console is attached
//                                       3 lit = a byte has arrived from the PC
//                                       5 lit = availableForWrite() > 0 (there is room in the transmit buffer)
//                                       12     = heartbeat (blinks twice a second)
// Change log
//   1.0  first version
// ===========================================================================================
#define TEST_VERSION "1.0"

#include <Arduino.h>
#if defined(ARDUINO_UNOR4_WIFI)
#include <Arduino_LED_Matrix.h>
ArduinoLEDMatrix matrix;
#endif

static const uint8_t kHex[16][7] = {
    {0x0E, 0x11, 0x13, 0x15, 0x19, 0x11, 0x0E}, {0x04, 0x0C, 0x04, 0x04, 0x04, 0x04, 0x0E},
    {0x0E, 0x11, 0x01, 0x02, 0x04, 0x08, 0x1F}, {0x1E, 0x01, 0x01, 0x0E, 0x01, 0x01, 0x1E},
    {0x02, 0x06, 0x0A, 0x12, 0x1F, 0x02, 0x02}, {0x1F, 0x10, 0x1E, 0x01, 0x01, 0x11, 0x0E},
    {0x06, 0x08, 0x10, 0x1E, 0x11, 0x11, 0x0E}, {0x1F, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08},
    {0x0E, 0x11, 0x11, 0x0E, 0x11, 0x11, 0x0E}, {0x0E, 0x11, 0x11, 0x0F, 0x01, 0x02, 0x0C},
    {0x0E, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11}, {0x1E, 0x11, 0x11, 0x1E, 0x11, 0x11, 0x1E},
    {0x0E, 0x11, 0x10, 0x10, 0x10, 0x11, 0x0E}, {0x1E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1E},
    {0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x1F}, {0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x10}};

static uint8_t stage = 0;              // 0..4 = A..E
static uint32_t stageStart = 0;
static uint32_t lastSend = 0;
static size_t lastAccepted = 0;        // bytes the core said it took for the last send
static bool everReceived = false;
static bool dtrCalled = false;

#if defined(ARDUINO_UNOR4_WIFI)
static void setPixel(uint32_t* frame, uint8_t row, uint8_t col) {
  const uint8_t index = static_cast<uint8_t>(row * 12 + col);
  frame[index / 32] |= 1UL << (31 - (index % 32));
}
static void drawGlyph(uint32_t* frame, uint8_t value, uint8_t leftColumn) {
  for (uint8_t row = 0; row < 7; ++row)
    for (uint8_t bit = 0; bit < 5; ++bit)
      if (kHex[value & 15][row] & (0x10 >> bit)) setPixel(frame, row, static_cast<uint8_t>(leftColumn + bit));
}
static void showMatrix(uint32_t now) {
  static uint32_t next = 0;
  static bool heartbeat = false;
  if (static_cast<int32_t>(now - next) < 0) return;
  next = now + 500;
  heartbeat = !heartbeat;
  uint32_t frame[3] = {0, 0, 0};
  drawGlyph(frame, static_cast<uint8_t>(10 + stage), 0);                        // A..E
  drawGlyph(frame, static_cast<uint8_t>(lastAccepted > 15 ? 15 : lastAccepted), 7);
  if (Serial) setPixel(frame, 7, 0);
  if (everReceived) setPixel(frame, 7, 2);
  if (Serial.availableForWrite() > 0) setPixel(frame, 7, 4);
  if (heartbeat) setPixel(frame, 7, 11);
  matrix.loadFrame(frame);
}
#endif

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);  // (the core has already started USB serial; this is harmless)
#if defined(ARDUINO_UNOR4_WIFI)
  matrix.begin();
#endif
  stageStart = millis();
}

void loop() {
  const uint32_t now = millis();
  digitalWrite(LED_BUILTIN, (now / 500) % 2);
#if defined(ARDUINO_UNOR4_WIFI)
  showMatrix(now);
#endif

  if (Serial.available() > 0) {  // note that something arrived (and swallow it)
    Serial.read();
    everReceived = true;
  }

  if (now - stageStart >= 8000) {
    stage = (stage + 1) % 5;
    stageStart = now;
  }

  if (now - lastSend >= 1000) {   // one test message per second
    lastSend = now;
    switch (stage) {
      case 0:  // A: normal println
        lastAccepted = Serial.println("A: println OK");
        break;
      case 1: {  // B: raw buffer write
        const char msg[] = "B: raw write OK\n";
        lastAccepted = Serial.write(reinterpret_cast<const uint8_t*>(msg), sizeof msg - 1);
        break;
      }
      case 2:  // C: force-connected mode
        if (!dtrCalled) { dtrCalled = true; Serial.dtr(); }
        lastAccepted = Serial.println("C: after dtr() OK");
        break;
      case 3:  // D: println + flush
        lastAccepted = Serial.println("D: println+flush OK");
        Serial.flush();
        break;
      case 4: {  // E: one byte at a time
        const char msg[] = "E: byte-by-byte OK\n";
        size_t total = 0;
        for (size_t i = 0; i < sizeof msg - 1; ++i) total += Serial.write(static_cast<uint8_t>(msg[i]));
        lastAccepted = total;
        break;
      }
    }
  }
}
