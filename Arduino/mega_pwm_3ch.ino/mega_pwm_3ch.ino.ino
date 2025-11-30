/* mega_pwm_3ch.ino
 * 3 PWM-канала: L (левая гусеница), R (правая), B (инструмент).
 * Только "вперёд": значения 0..255 -> duty cycle 0..100%.
 *
 * Протокол по USB-Serial (115200 бод):
 *   M L=<0..255> R=<0..255> B=<0..255>   // установить мощности
 *   PING                                   // ответ: PONG
 *
 * Без команд > SERIAL_TIMEOUT_MS -> аварийное обнуление ШИМ.
 */

#include <Arduino.h>

// --- Пины PWM на MEGA ---
// Выбрал 5, 6, 9 — гарантированно ШИМ на Mega2560 (Timer3/4/2).
const uint8_t PIN_L = 5;   // PWM -> EN левого драйвера
const uint8_t PIN_R = 6;   // PWM -> EN правого драйвера
const uint8_t PIN_B = 9;   // PWM -> EN инструмента (воздуходув и т.п.)

// Светодиод для индикации жизни
const uint8_t LED = LED_BUILTIN;

// Таймаут безопасности: если нет команд дольше этого времени — стоп (мс)
const unsigned long SERIAL_TIMEOUT_MS = 600;

// Текущие значения PWM (0..255)
uint8_t pwmL = 0, pwmR = 0, pwmB = 0;

// Временные переменные парсера
String inLine;
unsigned long lastCmdMs = 0;

static inline void applyPwm() {
  analogWrite(PIN_L, pwmL);
  analogWrite(PIN_R, pwmR);
  analogWrite(PIN_B, pwmB);
}

static inline void safeStopIfTimedOut() {
  if (millis() - lastCmdMs > SERIAL_TIMEOUT_MS) {
    if (pwmL || pwmR || pwmB) {
      pwmL = pwmR = pwmB = 0;
      applyPwm();
      Serial.println(F("OK TIMEOUT->STOP"));
    }
    lastCmdMs = millis(); // чтобы не флудить
  }
}

static bool parseUint(const String& tok, const char key, uint8_t &outVal) {
  // ожидаем формат "L=123" или "R=0" и т.п.
  int eq = tok.indexOf('=');
  if (eq < 1) return false;
  if (tok.charAt(0) != key) return false;
  long v = tok.substring(eq + 1).toInt();
  if (v < 0) v = 0;
  if (v > 255) v = 255;
  outVal = (uint8_t)v;
  return true;
}

static void handleLine(String line) {
  line.trim();
  if (line.length() == 0) return;

  // Унифицируем пробелы
  line.replace('\t', ' ');

  // PING?
  if (line.equalsIgnoreCase(F("PING"))) {
    Serial.println(F("PONG"));
    lastCmdMs = millis();
    return;
  }

  // Команда M ...
  // Допускаем разделители пробелами, например:
  // "M L=120 R=200 B=0"
  if (line.charAt(0) == 'M' || line.charAt(0) == 'm') {
    // Разобьём строку на токены
    uint8_t newL = pwmL, newR = pwmR, newB = pwmB;

    // Удалить первый символ 'M' и пробел (если есть)
    String rest = line.substring(1); rest.trim();

    int start = 0;
    while (start < rest.length()) {
      // найти следующий пробел
      int sp = rest.indexOf(' ', start);
      String tok = (sp == -1) ? rest.substring(start) : rest.substring(start, sp);
      tok.trim();
      if (tok.length()) {
        uint8_t tmp;
        if      (parseUint(tok, 'L', tmp)) newL = tmp;
        else if (parseUint(tok, 'R', tmp)) newR = tmp;
        else if (parseUint(tok, 'B', tmp)) newB = tmp;
        // игнорируем неизвестные токены
      }
      if (sp == -1) break;
      start = sp + 1;
    }

    pwmL = newL; pwmR = newR; pwmB = newB;
    applyPwm();
    lastCmdMs = millis();

    Serial.print(F("OK M L=")); Serial.print(pwmL);
    Serial.print(F(" R="));      Serial.print(pwmR);
    Serial.print(F(" B="));      Serial.println(pwmB);
    return;
  }

  // Неизвестная команда
  Serial.println(F("ERR"));
}

void setup() {
  pinMode(PIN_L, OUTPUT);
  pinMode(PIN_R, OUTPUT);
  pinMode(PIN_B, OUTPUT);
  pinMode(LED, OUTPUT);

  // Старт с нуля
  pwmL = pwmR = pwmB = 0;
  applyPwm();

  Serial.begin(115200);
  while (!Serial) { /* подождём USB (на некоторых системах) */ }

  lastCmdMs = millis();
  Serial.println(F("READY Mega2560 3PWM (L,R,B), 115200. Commands: 'PING' or 'M L= R= B='"));
}

void loop() {
  // простая "живая" индикация
  static unsigned long tblink = 0;
  if (millis() - tblink > 500) {
    tblink = millis();
    digitalWrite(LED, !digitalRead(LED));
  }

  // читать по строкам до '\n'
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

  // таймаут безопасности
  safeStopIfTimedOut();
}