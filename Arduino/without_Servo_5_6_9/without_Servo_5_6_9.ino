/* mega_esc_3ch_timer3.ino
 * 3 канала ESC/servo на Timer3:
 *   OC3A=pin5, OC3B=pin2, OC3C=pin3
 * Сигнал 50 Гц: 1000..2000us, нейтраль 1500us.
 *
 * Протокол:
 *   M L=... R=... B=...
 *   PING
 * Heartbeat: TICK ...
 * Failsafe: TIMEOUT->NEUTRAL (1500)
 */

#include <Arduino.h>

const uint8_t PIN_L = 5; // OC3A
const uint8_t PIN_R = 6; // OC3B
const uint8_t PIN_B = 9; // OC3C

const uint8_t LED = LED_BUILTIN;

static const uint16_t PWM_MIN = 1000;
static const uint16_t PWM_NEU = 1500;
static const uint16_t PWM_MAX = 2000;

static const unsigned long TIMEOUT_MS = 600;
static const unsigned long HB_MS      = 1000;

String inLine;
unsigned long lastCmdMs = 0;
unsigned long lastHbMs  = 0;
bool timedOutLatched = false;

uint16_t pwmL = PWM_NEU, pwmR = PWM_NEU, pwmB = PWM_NEU;

static inline uint16_t clampUs(long v) {
  if (v < PWM_MIN) return PWM_MIN;
  if (v > PWM_MAX) return PWM_MAX;
  return (uint16_t)v;
}

// Timer3: prescaler=8 => tick=0.5us (16MHz/8=2MHz)
// TOP for 20ms => 20000us / 0.5us = 40000
static inline uint16_t us_to_counts(uint16_t us) {
  return (uint16_t)(us * 2);
}

static inline void applyPwm() {
  OCR3A = us_to_counts(pwmL);
  OCR3B = us_to_counts(pwmR);
  OCR3C = us_to_counts(pwmB);
}

static void safeNeutral(const __FlashStringHelper* why) {
  pwmL = pwmR = pwmB = PWM_NEU;
  applyPwm();
  if (!timedOutLatched) {
    timedOutLatched = true;
    Serial.print(F("OK ")); Serial.println(why);
  }
}

static bool parseKV(const String& tok, char key, uint16_t &outUs) {
  int eq = tok.indexOf('=');
  if (eq < 1) return false;
  if (tok.charAt(0) != key) return false;
  long v = tok.substring(eq + 1).toInt();
  outUs = clampUs(v);
  return true;
}

static void handleLine(String line) {
  line.trim();
  if (!line.length()) return;

  line.replace('\t',' ');

  if (line.equalsIgnoreCase(F("PING"))) {
    Serial.println(F("PONG"));
    lastCmdMs = millis();
    timedOutLatched = false;
    return;
  }

  if (line.charAt(0) == 'M' || line.charAt(0) == 'm') {
    uint16_t newL = pwmL, newR = pwmR, newB = pwmB;

    String rest = line.substring(1); rest.trim();
    int start = 0;
    while (start < rest.length()) {
      int sp = rest.indexOf(' ', start);
      String tok = (sp == -1) ? rest.substring(start) : rest.substring(start, sp);
      tok.trim();
      if (tok.length()) {
        uint16_t tmp;
        if      (parseKV(tok, 'L', tmp)) newL = tmp;
        else if (parseKV(tok, 'R', tmp)) newR = tmp;
        else if (parseKV(tok, 'B', tmp)) newB = tmp;
      }
      if (sp == -1) break;
      start = sp + 1;
    }

    pwmL = newL; pwmR = newR; pwmB = newB;
    applyPwm();

    lastCmdMs = millis();
    timedOutLatched = false;

    Serial.print(F("OK M L=")); Serial.print(pwmL);
    Serial.print(F(" R="));      Serial.print(pwmR);
    Serial.print(F(" B="));      Serial.println(pwmB);
    return;
  }

  Serial.println(F("ERR"));
}

static void setup_timer3_50hz() {
  pinMode(PIN_L, OUTPUT);
  pinMode(PIN_R, OUTPUT);
  pinMode(PIN_B, OUTPUT);

  // Fast PWM, TOP=ICR3, non-inverting on A/B/C, prescaler=8
  TCCR3A = 0;
  TCCR3B = 0;

  // non-inverting outputs
  TCCR3A |= (1 << COM3A1) | (1 << COM3B1) | (1 << COM3C1);

  // WGM: mode 14 => Fast PWM, TOP=ICR3
  TCCR3A |= (1 << WGM31);
  TCCR3B |= (1 << WGM33) | (1 << WGM32);

  // prescaler = 8
  TCCR3B |= (1 << CS31);

  ICR3 = 40000; // 20ms period

  applyPwm();
}

void setup() {
  pinMode(LED, OUTPUT);

  Serial.begin(115200);
  while (!Serial) {}

  setup_timer3_50hz();
  safeNeutral(F("BOOT->NEUTRAL"));

  lastCmdMs = millis();
  lastHbMs = millis();

  Serial.println(F("READY Mega2560 ESC 3ch TIMER3 (pin5,2,3) 115200"));
}

void loop() {
  static unsigned long tblink = 0;
  if (millis() - tblink > 500) {
    tblink = millis();
    digitalWrite(LED, !digitalRead(LED));
  }

  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      handleLine(inLine);
      inLine = "";
    } else {
      if (inLine.length() < 120) inLine += c;
    }
  }

  if (millis() - lastCmdMs > TIMEOUT_MS) {
    safeNeutral(F("TIMEOUT->NEUTRAL"));
    lastCmdMs = millis();
  }

  if (millis() - lastHbMs > HB_MS) {
    lastHbMs = millis();
    Serial.print(F("TICK ms=")); Serial.print(millis());
    Serial.print(F(" L=")); Serial.print(pwmL);
    Serial.print(F(" R=")); Serial.print(pwmR);
    Serial.print(F(" B=")); Serial.println(pwmB);
  }
}
