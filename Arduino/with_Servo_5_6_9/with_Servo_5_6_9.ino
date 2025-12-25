/* mega_esc_3ch_servo.ino
 * 3 канала ESC/servo: L, R, B.
 * Сигнал: 1000..2000 us, 1500 us = нейтраль/стоп.
 *
 * Протокол (115200):
 *   M L=<1000..2000> R=<1000..2000> B=<1000..2000>
 *   PING
 *
 * Heartbeat:
 *   раз в HB_MS печатаем: "TICK ms=... L=... R=... B=..."
 *
 * Failsafe:
 *   если нет валидных команд > TIMEOUT_MS -> ставим 1500/1500/1500 и логируем 1 раз.
 */

#include <Arduino.h>
#include <Servo.h>

const uint8_t PIN_L = 5;
const uint8_t PIN_R = 6;
const uint8_t PIN_B = 9;

const uint8_t LED = LED_BUILTIN;

static const uint16_t PWM_MIN = 1000;
static const uint16_t PWM_NEU = 1500;
static const uint16_t PWM_MAX = 2000;

static const unsigned long TIMEOUT_MS = 600;  // safety timeout
static const unsigned long HB_MS      = 1000; // tick every 1s

Servo sL, sR, sB;

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

static inline void applyPwm() {
  sL.writeMicroseconds(pwmL);
  sR.writeMicroseconds(pwmR);
  sB.writeMicroseconds(pwmB);
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

void setup() {
  pinMode(LED, OUTPUT);

  Serial.begin(115200);
  while (!Serial) {}

  sL.attach(PIN_L);
  sR.attach(PIN_R);
  sB.attach(PIN_B);

  safeNeutral(F("BOOT->NEUTRAL"));

  lastCmdMs = millis();
  lastHbMs = millis();

  Serial.println(F("READY Mega2560 ESC 3ch SERVO (L,R,B) 115200"));
}

void loop() {
  // LED blink
  static unsigned long tblink = 0;
  if (millis() - tblink > 500) {
    tblink = millis();
    digitalWrite(LED, !digitalRead(LED));
  }

  // read lines
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

  // failsafe timeout
  if (millis() - lastCmdMs > TIMEOUT_MS) {
    safeNeutral(F("TIMEOUT->NEUTRAL"));
    lastCmdMs = millis(); // anti-spam latch controlled separately
  }

  // heartbeat tick
  if (millis() - lastHbMs > HB_MS) {
    lastHbMs = millis();
    Serial.print(F("TICK ms=")); Serial.print(millis());
    Serial.print(F(" L=")); Serial.print(pwmL);
    Serial.print(F(" R=")); Serial.print(pwmR);
    Serial.print(F(" B=")); Serial.println(pwmB);
  }
}
