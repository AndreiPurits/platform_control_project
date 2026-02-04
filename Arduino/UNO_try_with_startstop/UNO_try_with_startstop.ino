/*
UNO Rover RC PWM (1000..2000 us) — SIMPLE LOOP VERSION
+ START / STOP buttons via Serial events

PWM outputs (НЕ МЕНЯЕМ):
  D6 = Left
  D5 = Right
  D7 = Aux (B)

Buttons (NEW):
  D8 = START  (to GND)
  D9 = STOP   (to GND)
*/

#include <Arduino.h>

static const uint32_t BAUD = 115200;

// ===== PWM PINS (AS IS) =====
static const uint8_t PIN_L = 6;
static const uint8_t PIN_R = 5;
static const uint8_t PIN_B = 7;

// ===== BUTTON PINS (NEW) =====
static const uint8_t PIN_START = 8;
static const uint8_t PIN_STOP  = 9;

// ===== PWM CONST =====
static const int PWM_MIN = 1000;
static const int PWM_NEU = 1500;
static const int PWM_MAX = 2000;

static volatile int gL = PWM_NEU;
static volatile int gR = PWM_NEU;
static volatile int gB = PWM_NEU;

static inline int clamp_us(int v) {
 if (v < PWM_MIN) return PWM_MIN;
 if (v > PWM_MAX) return PWM_MAX;
 return v;
}

// -------- parser (AS IS) --------
static char line[96];
static uint8_t idx = 0;

static bool is_digit(char c){ return c >= '0' && c <= '9'; }

static bool parse_int_after(const char* s, int start, int& out) {
 int i = start;
 while (s[i] && !is_digit(s[i]) && s[i] != '-') i++;
 if (!s[i]) return false;

 bool neg = false;
 if (s[i] == '-') { neg = true; i++; }
 if (!is_digit(s[i])) return false;

 long v = 0;
 while (s[i] && is_digit(s[i])) {
   v = v * 10 + (s[i] - '0');
   i++;
 }
 out = neg ? (int)-v : (int)v;
 return true;
}

static bool parse_cmd(const char* s, int& L, int& R, int& B) {
 bool hasM=false, hasL=false, hasR=false, hasB=false;
 int l=PWM_NEU, r=PWM_NEU, b=PWM_NEU;

 for (int i=0; s[i]; i++) {
   if (s[i]=='M' || s[i]=='m') { hasM=true; break; }
 }
 if (!hasM) return false;

 for (int i=0; s[i]; i++) {
   if ((s[i]=='L'||s[i]=='l') && !hasL) { int v; if (parse_int_after(s,i+1,v)) { l=v; hasL=true; } }
   if ((s[i]=='R'||s[i]=='r') && !hasR) { int v; if (parse_int_after(s,i+1,v)) { r=v; hasR=true; } }
   if ((s[i]=='B'||s[i]=='b') && !hasB) { int v; if (parse_int_after(s,i+1,v)) { b=v; hasB=true; } }
 }

 if (!(hasL && hasR)) return false;

 L = clamp_us(l);
 R = clamp_us(r);
 B = hasB ? clamp_us(b) : PWM_NEU;
 return true;
}

// -------- PWM GENERATOR (AS IS) --------
static void pulse3(int L, int R, int B) {
 digitalWrite(PIN_L, HIGH);
 digitalWrite(PIN_R, HIGH);
 digitalWrite(PIN_B, HIGH);

 unsigned long t0 = micros();

 bool l_on=true, r_on=true, b_on=true;

 while (l_on || r_on || b_on) {
   unsigned long dt = micros() - t0;
   if (l_on && dt >= (unsigned long)L) { digitalWrite(PIN_L, LOW); l_on=false; }
   if (r_on && dt >= (unsigned long)R) { digitalWrite(PIN_R, LOW); r_on=false; }
   if (b_on && dt >= (unsigned long)B) { digitalWrite(PIN_B, LOW); b_on=false; }
 }

 while ((micros() - t0) < 20000UL) {
   // idle
 }
}

// ===== BUTTON STATE (NEW) =====
static bool lastStart = HIGH;
static bool lastStop  = HIGH;

void setup() {
 pinMode(PIN_L, OUTPUT);
 pinMode(PIN_R, OUTPUT);
 pinMode(PIN_B, OUTPUT);

 pinMode(PIN_START, INPUT_PULLUP);
 pinMode(PIN_STOP,  INPUT_PULLUP);

 digitalWrite(PIN_L, LOW);
 digitalWrite(PIN_R, LOW);
 digitalWrite(PIN_B, LOW);

 Serial.begin(BAUD);
 Serial.println("UNO_ROVER_RC_PWM_READY");
}

void loop() {
 // 1) PWM — КАК БЫЛО
 pulse3(gL, gR, gB);

 // 2) Serial — КАК БЫЛО
 while (Serial.available()) {
   char c = (char)Serial.read();
   if (c == '\r') continue;

   if (c == '\n') {
     line[idx] = 0;
     idx = 0;

     int nl, nr, nb;
     if (parse_cmd(line, nl, nr, nb)) {
       gL = nl; gR = nr; gB = nb;
       Serial.println("OK");
     }
   } else {
     if (idx < sizeof(line) - 1) line[idx++] = c;
     else idx = 0;
   }
 }

 // 3) BUTTONS — НОВОЕ, НЕ МЕШАЕТ PWM
 bool s = digitalRead(PIN_START);
 bool t = digitalRead(PIN_STOP);

 if (lastStart == HIGH && s == LOW) {
   Serial.println("START");
 }
 if (lastStop == HIGH && t == LOW) {
   Serial.println("STOP");
 }

 lastStart = s;
 lastStop  = t;
}
