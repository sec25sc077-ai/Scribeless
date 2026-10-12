/*
 * Assistive Handwriting System for Visually Impaired Individuals
 * ESP32 firmware
 *
 * Responsibilities
 *   - Overlay lift servo   : opens/closes the micro-grooved rubber writing overlay
 *   - Page-turner servo    : sweeps one page over
 *   - Positioning servo    : pushes the overlay back to its start position
 *   - Haptic pen motor     : PWM-driven vibration motor, triggered by the vision node
 *   - Command input        : USB-Serial (115200) and/or BLE (Nordic UART Service)
 *
 * Text protocol (one command per line, '\n' terminated, case-insensitive):
 *   PING                  -> PONG
 *   VIB <0-255> <ms>      -> vibrate at intensity for ms (auto-stops, ms clamped)
 *   VIB_OFF               -> stop vibration immediately
 *   LIFT_OPEN             -> raise overlay
 *   LIFT_CLOSE            -> lower overlay
 *   PAGE_TURN             -> full sequence: open -> turn page -> reposition -> close
 *   RESET_OVERLAY         -> reposition overlay only
 *   HOME                  -> all servos to home, vibration off
 *   SERVO <0|1|2> <deg>   -> raw servo move (for calibrating the angles below)
 *   STATUS                -> one-line status report
 * Replies: OK, PONG, BUSY, ERR <reason>, DONE <what>, STATUS ...
 *
 * Libraries: "ESP32Servo" (Kevin Harrington / John K. Bennett). BLE is part of the
 * arduino-esp32 core. Works with arduino-esp32 core 2.x and 3.x.
 *
 * Board: "ESP32 Dev Module". Power servos from a SEPARATE 5 V supply (common GND).
 */

#include <Arduino.h>
#include <ESP32Servo.h>

// ============================= USER CONFIGURATION ============================
#define ENABLE_BLE 1                       // 0 = USB-Serial only
#define BLE_DEVICE_NAME "AssistivePen-ESP32"

// ---- Pins ----
static const uint8_t PIN_SERVO_LIFT = 18;  // overlay lift
static const uint8_t PIN_SERVO_PAGE = 19;  // page-turn arm
static const uint8_t PIN_SERVO_POS  = 21;  // overlay positioner
static const uint8_t PIN_VIB        = 25;  // -> NPN/MOSFET gate driving the vibration motor
static const uint8_t PIN_LED        = 2;   // onboard LED (status)

// ---- Servo angles (degrees) : CALIBRATE with the SERVO command! ----
static const int LIFT_CLOSED_ANGLE = 10;
static const int LIFT_OPEN_ANGLE   = 95;
static const int PAGE_HOME_ANGLE   = 0;
static const int PAGE_TURN_ANGLE   = 160;
static const int POS_HOME_ANGLE    = 20;
static const int POS_PUSH_ANGLE    = 120;

// ---- Servo speeds (deg/s). Slow = quiet, gentle on paper. ----
static const float LIFT_SPEED = 90.0f;
static const float PAGE_SPEED = 120.0f;
static const float POS_SPEED  = 100.0f;

// ---- Haptic motor ----
static const uint32_t VIB_PWM_FREQ  = 20000;  // 20 kHz: inaudible
static const uint8_t  VIB_PWM_BITS  = 8;
static const uint8_t  VIB_MIN_DUTY  = 70;     // coin motors stall below this
static const uint16_t VIB_KICK_MS   = 30;     // full-power kick to overcome inertia
static const uint16_t VIB_MAX_MS    = 2000;   // safety clamp per command
static const bool     VIB_SUPPRESS_DURING_SEQUENCE = true;
// =============================================================================

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  #define VIB_PWM_SETUP()  ledcAttach(PIN_VIB, VIB_PWM_FREQ, VIB_PWM_BITS)
  #define VIB_PWM_WRITE(d) ledcWrite(PIN_VIB, (d))
#else
  static const uint8_t VIB_PWM_CH = 8;        // high channel to avoid ESP32Servo's
  #define VIB_PWM_SETUP()  do { ledcSetup(VIB_PWM_CH, VIB_PWM_FREQ, VIB_PWM_BITS); \
                                ledcAttachPin(PIN_VIB, VIB_PWM_CH); } while (0)
  #define VIB_PWM_WRITE(d) ledcWrite(VIB_PWM_CH, (d))
#endif

#if ENABLE_BLE
  #include <BLEDevice.h>
  #include <BLEServer.h>
  #include <BLEUtils.h>
  #include <BLE2902.h>
  #define NUS_SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
  #define NUS_RX_UUID      "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"  // client -> ESP32
  #define NUS_TX_UUID      "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"  // ESP32 -> client (notify)
#endif

// ------------------------------- Smooth servo --------------------------------
// Non-blocking servo that ramps toward its target at a fixed angular speed, so
// the main loop (serial parsing, vibration timing) is never stalled.
struct SmoothServo {
  Servo servo;
  float cur = 0, target = 0, speed = 90;    // deg, deg, deg/s
  uint32_t lastMs = 0;

  void begin(uint8_t pin, float startAngle) {
    servo.setPeriodHertz(50);
    servo.attach(pin, 500, 2400);           // pulse range in us; adjust for your servo
    cur = target = startAngle;
    servo.write((int)cur);
    lastMs = millis();
  }
  void moveTo(float angle, float degPerSec) {
    target = constrain(angle, 0.0f, 180.0f);
    speed = degPerSec;
  }
  void update() {
    uint32_t now = millis();
    float step = speed * (now - lastMs) / 1000.0f;
    lastMs = now;
    if (cur == target) return;
    if (fabsf(target - cur) <= step) cur = target;
    else cur += (target > cur) ? step : -step;
    servo.write((int)roundf(cur));
  }
  bool atTarget() const { return cur == target; }
};

enum ServoId : int8_t { SV_LIFT = 0, SV_PAGE = 1, SV_POS = 2 };
SmoothServo servos[3];
const float SERVO_SPEEDS[3] = { LIFT_SPEED, PAGE_SPEED, POS_SPEED };

// ------------------------------ Motion sequences -----------------------------
// A sequence is a list of (servo, angle, dwell). A step completes when the servo
// reaches its angle and the dwell time has elapsed.
struct Step { int8_t servo; int16_t angle; uint16_t dwellMs; };

static const Step SEQ_PAGE_TURN[] = {
  { SV_LIFT, LIFT_OPEN_ANGLE,   300 },   // 1. raise the overlay clear of the page
  { SV_PAGE, PAGE_TURN_ANGLE,   400 },   // 2. sweep the page over
  { SV_PAGE, PAGE_HOME_ANGLE,   300 },   // 3. retract the arm
  { SV_POS,  POS_PUSH_ANGLE,    300 },   // 4. push overlay back to start position
  { SV_POS,  POS_HOME_ANGLE,    300 },   // 5. retract positioner
  { SV_LIFT, LIFT_CLOSED_ANGLE, 200 },   // 6. lower overlay onto the new page
};
static const Step SEQ_RESET_OVERLAY[] = {
  { SV_POS,  POS_PUSH_ANGLE, 300 },
  { SV_POS,  POS_HOME_ANGLE, 300 },
};

const Step* seq = nullptr;
uint8_t seqLen = 0, seqIdx = 0;
bool seqStepIssued = false;
uint32_t seqDwellUntil = 0;
const char* seqName = "";

bool sequenceBusy() { return seq != nullptr; }

void startSequence(const Step* s, uint8_t len, const char* name) {
  seq = s; seqLen = len; seqIdx = 0; seqStepIssued = false; seqName = name;
}

void updateSequence();                 // forward declarations
void reply(const String& msg);

// --------------------------------- Haptics -----------------------------------
bool vibActive = false;
uint32_t vibEndMs = 0, vibKickEndMs = 0;
uint8_t vibDuty = 0;

void vibStop() { VIB_PWM_WRITE(0); vibActive = false; }

void vibStart(uint8_t intensity, uint16_t ms) {
  if (intensity == 0 || ms == 0) { vibStop(); return; }
  ms = min<uint16_t>(ms, VIB_MAX_MS);
  // Map 1..255 -> VIB_MIN_DUTY..255 so low intensities still spin the motor.
  vibDuty = map(intensity, 1, 255, VIB_MIN_DUTY, 255);
  uint32_t now = millis();
  if (!vibActive) { VIB_PWM_WRITE(255); vibKickEndMs = now + VIB_KICK_MS; }  // kick-start
  else if (now >= vibKickEndMs) VIB_PWM_WRITE(vibDuty);
  vibEndMs = now + ms;
  vibActive = true;
}

void updateVibration() {
  if (!vibActive) return;
  uint32_t now = millis();
  if ((int32_t)(now - vibEndMs) >= 0) { vibStop(); return; }       // timed out = fail-safe
  if (now >= vibKickEndMs) VIB_PWM_WRITE(vibDuty);
}

// ---------------------------- Command transport ------------------------------
// Both transports feed complete lines into handleCommand().
String serialBuf;

#if ENABLE_BLE
  BLEServer* bleServer = nullptr;
  BLECharacteristic* bleTx = nullptr;
  volatile bool bleConnected = false;

  // Single-producer (BLE task) / single-consumer (loop) ring buffer.
  static const uint16_t BLE_RING = 512;
  volatile char bleRing[BLE_RING];
  volatile uint16_t bleHead = 0, bleTail = 0;
  String bleBuf;

  class ServerCb : public BLEServerCallbacks {
    void onConnect(BLEServer*) override { bleConnected = true; }
    void onDisconnect(BLEServer*) override {
      bleConnected = false;
      vibStop();                                   // fail-safe: link lost
      BLEDevice::startAdvertising();
    }
  };
  class RxCb : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* c) override {
      auto v = c->getValue();                      // std::string (2.x) or String (3.x)
      for (size_t i = 0; i < v.length(); i++) {
        uint16_t next = (bleHead + 1) % BLE_RING;
        if (next == bleTail) break;                // overflow: drop
        bleRing[bleHead] = v[i];
        bleHead = next;
      }
    }
  };

  void bleBegin() {
    BLEDevice::init(BLE_DEVICE_NAME);
    bleServer = BLEDevice::createServer();
    bleServer->setCallbacks(new ServerCb());
    BLEService* svc = bleServer->createService(NUS_SERVICE_UUID);
    bleTx = svc->createCharacteristic(NUS_TX_UUID, BLECharacteristic::PROPERTY_NOTIFY);
    bleTx->addDescriptor(new BLE2902());
    BLECharacteristic* rx = svc->createCharacteristic(
        NUS_RX_UUID, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
    rx->setCallbacks(new RxCb());
    svc->start();
    BLEAdvertising* adv = BLEDevice::getAdvertising();
    adv->addServiceUUID(NUS_SERVICE_UUID);
    adv->setScanResponse(true);
    BLEDevice::startAdvertising();
  }
#endif

void reply(const String& msg) {
  Serial.println(msg);
#if ENABLE_BLE
  if (bleConnected && bleTx) {
    String m = msg + "\n";
    bleTx->setValue((uint8_t*)m.c_str(), m.length());
    bleTx->notify();
  }
#endif
}

// ------------------------------ Command handling -----------------------------
void homeAll() {
  vibStop();
  servos[SV_LIFT].moveTo(LIFT_CLOSED_ANGLE, LIFT_SPEED);
  servos[SV_PAGE].moveTo(PAGE_HOME_ANGLE, PAGE_SPEED);
  servos[SV_POS].moveTo(POS_HOME_ANGLE, POS_SPEED);
}

void handleCommand(String line) {
  line.trim();
  if (line.isEmpty()) return;
  line.toUpperCase();

  // Split "CMD arg1 arg2"
  int sp1 = line.indexOf(' ');
  String cmd = (sp1 < 0) ? line : line.substring(0, sp1);
  String rest = (sp1 < 0) ? "" : line.substring(sp1 + 1);
  rest.trim();

  if (cmd == "PING") { reply("PONG"); return; }

  if (cmd == "VIB") {
    int a = 0, b = 0;
    if (sscanf(rest.c_str(), "%d %d", &a, &b) != 2 || a < 0 || a > 255 || b < 0) {
      reply("ERR VIB_ARGS"); return;
    }
    if (VIB_SUPPRESS_DURING_SEQUENCE && sequenceBusy()) { reply("BUSY"); return; }
    vibStart((uint8_t)a, (uint16_t)min(b, 65535));
    reply("OK");                                   // (vision node may ignore OKs)
    return;
  }
  if (cmd == "VIB_OFF") { vibStop(); reply("OK"); return; }

  if (cmd == "STATUS") {
    reply(String("STATUS busy=") + (sequenceBusy() ? seqName : "none") +
          " lift=" + (int)servos[SV_LIFT].cur + " page=" + (int)servos[SV_PAGE].cur +
          " pos=" + (int)servos[SV_POS].cur + " vib=" + (vibActive ? 1 : 0));
    return;
  }

  if (cmd == "HOME") {
    seq = nullptr;
    homeAll();
    reply("OK");
    return;
  }

  // Everything below moves mechanics: refuse while a sequence is running.
  if (sequenceBusy()) { reply("BUSY"); return; }

  if (cmd == "LIFT_OPEN")  { servos[SV_LIFT].moveTo(LIFT_OPEN_ANGLE, LIFT_SPEED);   reply("OK"); return; }
  if (cmd == "LIFT_CLOSE") { servos[SV_LIFT].moveTo(LIFT_CLOSED_ANGLE, LIFT_SPEED); reply("OK"); return; }

  if (cmd == "PAGE_TURN") {
    vibStop();
    startSequence(SEQ_PAGE_TURN, sizeof(SEQ_PAGE_TURN) / sizeof(Step), "PAGE_TURN");
    reply("OK");
    return;
  }
  if (cmd == "RESET_OVERLAY") {
    startSequence(SEQ_RESET_OVERLAY, sizeof(SEQ_RESET_OVERLAY) / sizeof(Step), "RESET_OVERLAY");
    reply("OK");
    return;
  }
  if (cmd == "SERVO") {
    int id = -1, deg = -1;
    if (sscanf(rest.c_str(), "%d %d", &id, &deg) != 2 || id < 0 || id > 2 || deg < 0 || deg > 180) {
      reply("ERR SERVO_ARGS"); return;
    }
    servos[id].moveTo(deg, SERVO_SPEEDS[id]);
    reply("OK");
    return;
  }

  reply("ERR UNKNOWN_CMD");
}

// ------------------------------ Sequence engine ------------------------------
void updateSequence() {
  if (!seq) return;
  uint32_t now = millis();

  if (!seqStepIssued) {                            // start the next step
    const Step& s = seq[seqIdx];
    servos[s.servo].moveTo(s.angle, SERVO_SPEEDS[s.servo]);
    seqStepIssued = true;
    seqDwellUntil = 0;
    return;
  }

  const Step& s = seq[seqIdx];
  if (!servos[s.servo].atTarget()) return;         // still moving

  if (seqDwellUntil == 0) { seqDwellUntil = now + s.dwellMs; return; }
  if ((int32_t)(now - seqDwellUntil) < 0) return;  // dwelling

  seqIdx++;
  seqStepIssued = false;
  if (seqIdx >= seqLen) {                          // finished
    String done = String("DONE ") + seqName;
    seq = nullptr;
    reply(done);
  }
}

// ---------------------------------- Input ------------------------------------
void pollInputs() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') { handleCommand(serialBuf); serialBuf = ""; }
    else if (serialBuf.length() < 64) serialBuf += c;
  }
#if ENABLE_BLE
  while (bleTail != bleHead) {
    char c = bleRing[bleTail];
    bleTail = (bleTail + 1) % BLE_RING;
    if (c == '\n' || c == '\r') { handleCommand(bleBuf); bleBuf = ""; }
    else if (bleBuf.length() < 64) bleBuf += c;
  }
#endif
}

// ---------------------------------- Arduino ----------------------------------
void setup() {
  Serial.begin(115200);
  pinMode(PIN_LED, OUTPUT);

  VIB_PWM_SETUP();                                 // set up vibration PWM before servos
  VIB_PWM_WRITE(0);

  ESP32PWM::allocateTimer(2);                      // keep servo timers away from vib PWM
  ESP32PWM::allocateTimer(3);
  servos[SV_LIFT].begin(PIN_SERVO_LIFT, LIFT_CLOSED_ANGLE);
  servos[SV_PAGE].begin(PIN_SERVO_PAGE, PAGE_HOME_ANGLE);
  servos[SV_POS].begin(PIN_SERVO_POS, POS_HOME_ANGLE);

#if ENABLE_BLE
  bleBegin();
#endif

  Serial.println("READY");
  digitalWrite(PIN_LED, HIGH);
}

void loop() {
  pollInputs();
  for (auto& s : servos) s.update();
  updateSequence();
  updateVibration();

  // LED: on = idle, blinking = sequence running
  digitalWrite(PIN_LED, sequenceBusy() ? ((millis() / 150) & 1) : HIGH);
}
