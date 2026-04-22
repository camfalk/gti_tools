/*
 * Nissan Leaf Gen2 Motor → Volvo EPS Pump + RPM Gauge Controller
 *
 * Hardware : Adafruit Feather M4 CAN Express (ATSAMD51)
 * Platform : PlatformIO – see platformio.ini for board and lib_deps
 *
 * Wiring
 *   CAN bus  – shared 500 kbps bus carrying Leaf EV-CAN and Volvo EPS
 *   Servo    – signal wire → SERVO_PIN (PWM)
 *   LEDs     – anode → LED_LOW_PIN / LED_MED_PIN / LED_HIGH_PIN through 330 Ω resistors
 *
 * Failure modes
 *   - Arduino loses power: keep-alive + speed messages stop → pump falls back
 *     to its internal ~70% limp-home output (built-in behaviour, not zero).
 *   - Leaf CAN lost (Arduino still running): after RPM_TIMEOUT_MS both EPS
 *     messages are suppressed → pump again falls back to ~70% limp-home.
 *     All LEDs go dark as a fault indicator. Resumes normal control as soon
 *     as valid Leaf frames return.
 */

#include <Arduino.h>
#include <CANSAME5x.h>
#include <Servo.h>

// ── Pin Assignments ────────────────────────────────────────────────────────
#define SERVO_PIN     9   // PWM – RPM gauge needle
#define LED_LOW_PIN   10  // 1 LED  → low pump speed  (high motor RPM)
#define LED_MED_PIN   11  // 2 LEDs → mid pump speed
#define LED_HIGH_PIN  12  // 3 LEDs → high pump speed (low motor RPM)

// ── CAN IDs ────────────────────────────────────────────────────────────────
#define LEAF_RPM_CAN_ID       0x1DAUL       // Standard 11-bit  – Nissan Leaf motor speed
#define EPS_KEEPALIVE_CAN_ID  0x1AE0092CUL  // Extended 29-bit  – Volvo EPS heartbeat
#define EPS_SPEED_CAN_ID      0x2104136UL   // Extended 29-bit  – Volvo EPS pump speed

// ── Timing ────────────────────────────────────────────────────────────────
#define KEEPALIVE_MS         500  // 2 Hz  – EPS keep-alive period
#define SPEED_MSG_MS          14  // ~70 Hz – EPS speed control period
#define RPM_TIMEOUT_MS      2000  // Leaf CAN loss → suppress EPS messages
#define PUMP_IDLE_TIMEOUT_MS 4000 // RPM at zero this long → drop to minimum pump speed
#define RPM_DEADBAND          50  // RPM readings below this are treated as zero for idle timing

// ── Pump Speed Map ─────────────────────────────────────────────────────────
// Pump CAN value:  1 = maximum output,  6000 = minimum output
// Desired behaviour: lower motor RPM → higher pump output
#define PUMP_VAL_MAX     1     // full pump output (low/stopped motor RPM)
#define PUMP_VAL_MIN  6000     // minimum pump output (high motor RPM)
#define PUMP_VAL_IDLE 6000     // pump speed when motor has been at 0 RPM for PUMP_IDLE_TIMEOUT_MS

// ── Gauge Needle Smoothing ─────────────────────────────────────────────────
// Exponential moving average applied to the gauge only (pump uses raw RPM).
// Range 0.0–1.0: lower = smoother but slower, higher = snappier but jitterier.
#define RPM_SMOOTH_ALPHA 0.15f

// ── Gauge Servo Range ──────────────────────────────────────────────────────
// Stock gauge sweeps 180° from 0 to 7 000 RPM (redline).
// Motor 0–10 000 RPM maps linearly across that full sweep:
//   motor 0      → 0°   → gauge reads 0
//   motor 10 000 → 180° → gauge reads 7 000 (redline)
// Fine-tune these two values if the needle doesn't sit precisely at rest/redline.
#define SERVO_DEG_ZERO      0   // servo angle when motor is at 0 RPM
#define SERVO_DEG_FULLSCALE 180 // servo angle when motor is at 10 000 RPM

// ── LED Thresholds (motor RPM) ─────────────────────────────────────────────
#define RPM_THRESH_LOW  3334  // below → 3 LEDs (high pump)
#define RPM_THRESH_MED  6667  // below → 2 LEDs,  above → 1 LED (low pump)

// ── Debug Serial Output ────────────────────────────────────────────────────
// 0 = silent, 1 = periodic status summary, 2 = summary + full CAN sniffer
// Set to 2 for initial bring-up, drop to 1 once things are working, 0 for production.
#define DEBUG_MODE        2
#define DEBUG_INTERVAL_MS 500  // how often the status summary prints

// ── Globals ────────────────────────────────────────────────────────────────
CANSAME5x CAN;
Servo     rpmServo;

int16_t       motorRPM       = 0;
float         smoothedRPM    = 0.0f;  // EMA of abs motor RPM, drives gauge only
bool          leafCanActive  = false; // true once first valid Leaf frame arrives
uint8_t       keepAliveCounter = 0x00; // cycles 0x00 → 0x40 → 0x80 → 0xC0
unsigned long lastLeafMsgMs    = 0;
unsigned long lastNonZeroRpmMs = 0;
unsigned long lastKeepAliveMs  = 0;
unsigned long lastSpeedMsgMs   = 0;
unsigned long lastDebugMs      = 0;

uint16_t      lastPumpValue  = PUMP_VAL_IDLE;
bool          pumpIdling     = true;

// ── LED Indicator ──────────────────────────────────────────────────────────
void updateLEDs(int absRPM) {
  bool low  = false, med  = false, high = false;

  if (absRPM < RPM_THRESH_LOW) {
    low = med = high = true;  // high pump speed – all 3 LEDs
  } else if (absRPM < RPM_THRESH_MED) {
    low = med = true;         // mid pump speed – 2 LEDs
  } else {
    low = true;               // low pump speed – 1 LED
  }

  digitalWrite(LED_LOW_PIN,  low  ? HIGH : LOW);
  digitalWrite(LED_MED_PIN,  med  ? HIGH : LOW);
  digitalWrite(LED_HIGH_PIN, high ? HIGH : LOW);
}

// ── Servo / Gauge ──────────────────────────────────────────────────────────
void updateRPMGauge(float rpm) {
  rpm = constrain(rpm, 0.0f, 10000.0f);
  int angle = (int)((rpm / 10000.0f) * (SERVO_DEG_FULLSCALE - SERVO_DEG_ZERO) + SERVO_DEG_ZERO);
  rpmServo.write(angle);
}

// ── CAN TX: EPS keep-alive (2 Hz) ─────────────────────────────────────────
void sendEPSKeepAlive() {
  CAN.beginExtendedPacket(EPS_KEEPALIVE_CAN_ID);
  CAN.write(keepAliveCounter); // cycling counter
  CAN.write(0x00);
  CAN.write(0x22);
  CAN.write(0xE0);
  CAN.write(0x41);
  CAN.write(0x90);
  CAN.write(0x00);
  CAN.write(0x00);
  CAN.endPacket();

  keepAliveCounter = (keepAliveCounter + 0x40) & 0xFF;
}

// ── CAN TX: EPS speed control (~70 Hz) ────────────────────────────────────
// pumpValue: PUMP_VAL_MAX (1) = full output, PUMP_VAL_MIN (6000) = minimum output
void sendEPSSpeedControl(uint16_t pumpValue) {
  CAN.beginExtendedPacket(EPS_SPEED_CAN_ID);
  CAN.write(0xBB);
  CAN.write(0x00);
  CAN.write(0x3F);
  CAN.write(0xFF);
  CAN.write(0x06);
  CAN.write(0xE0);
  CAN.write((uint8_t)(pumpValue >> 8));
  CAN.write((uint8_t)(pumpValue & 0xFF));
  CAN.endPacket();
}

uint16_t rpmToPumpValue(int16_t rpm) {
  int absRPM = constrain(abs(rpm), 0, 10000);
  // Lower motor RPM → lower pump value number → higher pump output
  return (uint16_t)map(absRPM, 0, 10000, PUMP_VAL_MAX, PUMP_VAL_MIN);
}

// ── Debug ──────────────────────────────────────────────────────────────────
#if DEBUG_MODE >= 1
void printDebugSummary() {
  Serial.print("[STATUS] RPM=");
  Serial.print(motorRPM);
  Serial.print("  smooth=");
  Serial.print((int)smoothedRPM);
  Serial.print("  pump=");
  Serial.print(lastPumpValue);
  Serial.print("  state=");
  if      (!leafCanActive) Serial.println("FAULT – no Leaf CAN");
  else if  (pumpIdling)    Serial.println("idle");
  else                     Serial.println("active");
}
#endif

// ── Setup ──────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  // Feather M4 CAN Express: enable the on-board CAN transceiver
  pinMode(PIN_CAN_STANDBY, OUTPUT);
  digitalWrite(PIN_CAN_STANDBY, LOW);   // disable standby (active-low)
  pinMode(PIN_CAN_BOOSTEN, OUTPUT);
  digitalWrite(PIN_CAN_BOOSTEN, HIGH);  // enable 5 V boost rail for transceiver

  if (!CAN.begin(500000)) {
    Serial.println("CAN init failed – check wiring");
    while (1);
  }
  Serial.println("CAN ready @ 500 kbps");

  // Pump and Arduino power up together on ignition; wait for pump to finish
  // booting before we start sending CAN frames or it may miss early keep-alives
  // and stay in limp-home mode.
  Serial.println("Waiting for EPS pump boot...");
  delay(2000);
  Serial.println("Starting control loop");

  // LEDs
  pinMode(LED_LOW_PIN,  OUTPUT);
  pinMode(LED_MED_PIN,  OUTPUT);
  pinMode(LED_HIGH_PIN, OUTPUT);
  updateLEDs(0); // all LEDs on at startup (pump at max, motor at 0)

  // Servo – park needle at zero before anything moves
  rpmServo.attach(SERVO_PIN);
  rpmServo.write(SERVO_DEG_ZERO);
}

// ── Loop ───────────────────────────────────────────────────────────────────
void loop() {
  unsigned long now = millis();

  // ── RX: CAN bus ──────────────────────────────────────────────────────────
  int packetSize = CAN.parsePacket();
  if (packetSize && !CAN.packetRtr()) {
    long    id       = CAN.packetId();
    bool    extended = CAN.packetExtended();
    uint8_t data[8]  = {0};
    int     len      = 0;

    while (CAN.available() && len < 8) data[len++] = (uint8_t)CAN.read();

#if DEBUG_MODE >= 2
    Serial.print(extended ? "EXT 0x" : "STD 0x");
    Serial.print(id, HEX);
    Serial.print(" ["); Serial.print(len); Serial.print("]");
    for (int i = 0; i < len; i++) {
      Serial.print(" ");
      if (data[i] < 0x10) Serial.print("0");
      Serial.print(data[i], HEX);
    }
    Serial.println();
#endif

    if (id == LEAF_RPM_CAN_ID && !extended && len >= 6) {
      // Bytes 4–5: signed raw value; divide by 2 for actual RPM
      int16_t rawRPM = (int16_t)(((uint16_t)data[4] << 8) | data[5]);
      motorRPM      = rawRPM / 2;
      lastLeafMsgMs = now;
      leafCanActive = true;

      int absRPM = abs(motorRPM);
      if (absRPM > RPM_DEADBAND) lastNonZeroRpmMs = now;
      smoothedRPM = RPM_SMOOTH_ALPHA * absRPM + (1.0f - RPM_SMOOTH_ALPHA) * smoothedRPM;
      updateRPMGauge(smoothedRPM);
      updateLEDs(absRPM);
    }
  }

  // ── Leaf CAN timeout → suppress EPS messages, let pump fall to ~70% ────
  if (leafCanActive && (now - lastLeafMsgMs) > RPM_TIMEOUT_MS) {
    leafCanActive = false;
    motorRPM      = 0;
    smoothedRPM   = 0.0f;
    updateRPMGauge(0.0f);
    digitalWrite(LED_LOW_PIN,  LOW); // all LEDs off = fault indicator
    digitalWrite(LED_MED_PIN,  LOW);
    digitalWrite(LED_HIGH_PIN, LOW);
    Serial.println("Leaf CAN timeout – EPS messages suppressed, pump → ~70% limp-home");
  }

  // ── TX: EPS keep-alive and speed control (only while Leaf CAN is active) ─
  if (leafCanActive) {
    if (now - lastKeepAliveMs >= KEEPALIVE_MS) {
      sendEPSKeepAlive();
      lastKeepAliveMs = now;
    }
    if (now - lastSpeedMsgMs >= SPEED_MSG_MS) {
      pumpIdling    = (now - lastNonZeroRpmMs) > PUMP_IDLE_TIMEOUT_MS;
      lastPumpValue = pumpIdling ? PUMP_VAL_IDLE : rpmToPumpValue(motorRPM);
      // TODO: if pump surges when coming out of idle (jumps from PUMP_VAL_IDLE to
      // commanded value in one step), add a slew rate limit here — e.g. max change
      // of 200 per 14 ms cycle gives ~400 ms ramp from min to max.
      sendEPSSpeedControl(lastPumpValue);
      lastSpeedMsgMs = now;
    }
  }

#if DEBUG_MODE >= 1
  if (now - lastDebugMs >= DEBUG_INTERVAL_MS) {
    printDebugSummary();
    lastDebugMs = now;
  }
#endif
}
