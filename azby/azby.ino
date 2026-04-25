#include <QTRSensors.h>

QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount] = { 0 };

// Piny pre motory (L298N alebo podobný driver)
#define LEFT_FWD  6
#define LEFT_BWD  5
#define RIGHT_FWD 10
#define RIGHT_BWD 9
#define Tlak 11 // Nárazník / Tlakový senzor

int stav = 1;
long casstartu = 0;
int last_position = 3500;
int last_error = 0;

// ======================================================
// ==================== DOLADENIE =======================
// ======================================================
int IgnoreTime         = 2;    // Sekundy ignorovania nárazníka po štarte

int baseSpeed          = 120;   // Základná rýchlosť (0-255)
float kp               = 0.06; // Proporcionálna zložka
float kd               = 0.5;   // Derivačná zložka

int rych_tocenia       = 120;  // Rýchlosť pri ostrých zákrutách (rotácia)
int tehla              = 150;  // Rýchlosť pri obchádzaní prekážky

int sharpTurnThreshold = 50;   // Prah pre spomalenie v zákrute
int minTurnSpeed       = 40;   // Minimálna rýchlosť v prudkej zákrute

// ── COLOUR THRESHOLD ──────────────────────────────────
// Black line  → ~500-1000   (absorbs most IR)
// Coloured line (red/blue/green) → ~200-500  (absorbs some IR)
// White surface  → ~0-150   (reflects almost all IR)
//
// Set this just above the white surface reading.
// Lower value = follows lighter / more reflective coloured lines.
// Start with 200 for red/blue; drop to 150 if the line is lost often.
int lineThreshold = 200;
// ──────────────────────────────────────────────────────

// Časy pre manévre (v milisekundách)
int T_TURN_90   = 260;
int T_OBIST_BOK = 280;
int T_OBIST_DLZ = 680;

int limitSlepejJazdy  = 5000;  // Ako dlho ísť rovno pri strate čiary

// ======================================================
// ==================== MOTORY ==========================
// ======================================================
void setMotors(int left, int right) {
  left  = constrain(left,  -255, 255);
  right = constrain(right, -255, 255);

  analogWrite(LEFT_FWD,  left  > 0 ? left  : 0);
  analogWrite(LEFT_BWD,  left  < 0 ? -left : 0);
  analogWrite(RIGHT_FWD, right > 0 ? right : 0);
  analogWrite(RIGHT_BWD, right < 0 ? -right : 0);
}

void zastavsa()               { setMotors(0, 0); }
void tocVpravo(int a, int ms) { setMotors(a, -a); delay(ms); zastavsa(); }
void tocVlavo(int a, int ms)  { setMotors(-a, a); delay(ms); zastavsa(); }
void idDopredu(int r, int ms) { setMotors(r, r); delay(ms); zastavsa(); }

// ======================================================
// ====== POMOCNÁ FUNKCIA: JE ČIARA VIDITEĽNÁ? ==========
// ======================================================
// Works for any line colour: threshold is now 'lineThreshold'
// instead of a hardcoded 500, so red/blue/green lines are detected.
bool lineVisible() {
  for (uint8_t i = 0; i < SensorCount; i++) {
    if (sensorValues[i] > lineThreshold) return true;
  }
  return false;
}

// ======================================================
// ======= ADAPTÍVNA RÝCHLOSŤ — KVADRATICKÝ POKLES ======
// ======================================================
int getAdaptiveSpeed(int error) {
  int absError = abs(error);
  if (absError <= sharpTurnThreshold) return baseSpeed;

  float t = (float)(absError - sharpTurnThreshold) / (3500 - sharpTurnThreshold);
  t = constrain(t, 0.0f, 1.0f);
  return (int)(baseSpeed - (baseSpeed - minTurnSpeed) * t * t);
}

// ======================================================
// ========= PD REGULÁTOR - JADRO ROBOTA ================
// ======================================================
void pidStep() {
  uint16_t position = qtr.readLineBlack(sensorValues);

  if (!lineVisible()) {
    // ── STRATA ČIARY: pokračuj s poslednou korekciou ──────────
    // Ak sme sa točili vpravo pred stratou → stále sa toč vpravo
    // Ak sme boli rovne → ide rovno
    int correction = (int)(kp * last_error * 3.8f);  // mierne zosilni
    int speed      = baseSpeed;
    setMotors(speed - correction, speed + correction);
    return;
  }

  // ── normálny PD výpočet ────────────────────────────────────
  int error      = (int)position - 3500;
  int correction = (int)(kp * error) + (int)(kd * (error - last_error));
  int speed      = getAdaptiveSpeed(error);

  setMotors(speed - correction, speed + correction);

  last_position = position;
  last_error    = error;
}

// ======================================================
// ============= VYHYBANIE SA TEHLE =====================
// ======================================================
void vyhybajSaTehle() {
  setMotors(-tehla, -tehla);
  delay(150);
  zastavsa();
  delay(100);

  tocVpravo(tehla, T_TURN_90); delay(100);
  idDopredu(tehla, T_OBIST_BOK); delay(100);
  tocVlavo(tehla, 320); delay(100);
  idDopredu(tehla, T_OBIST_DLZ); delay(100);
  tocVlavo(tehla, T_TURN_90); delay(100);

  // Hľadanie čiary po obídení
  unsigned long timeout = millis();
  while (millis() - timeout < 3000) {
    qtr.readLineBlack(sensorValues);
    if (lineVisible()) {
      uint16_t pos = qtr.readLineBlack(sensorValues);
      if (pos > 1000 && pos < 6000) break;
    }
    setMotors(tehla, tehla);
  }
  zastavsa();
  delay(100);

  tocVpravo(tehla, 100);

  last_position = 3500;
  last_error    = 0;
}

// ======================================================
// ====================== SETUP =========================
// ======================================================
void setup() {
  Serial.begin(115200);

  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){ 15, 16, 17, 2, 3, 4, 7, 8 }, SensorCount);
  qtr.setEmitterPin(12);

  pinMode(LEFT_FWD,    OUTPUT);
  pinMode(LEFT_BWD,    OUTPUT);
  pinMode(RIGHT_FWD,   OUTPUT);
  pinMode(RIGHT_BWD,   OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(Tlak, INPUT_PULLUP);

  zastavsa();
  stav = 1;
  delay(2000);
}

// ======================================================
// ====================== LOOP ==========================
// ======================================================
void loop() {

  if (stav == 1) {
    Serial.println("Kalibrácia spustená...");
    digitalWrite(LED_BUILTIN, HIGH);
    for (uint16_t i = 0; i < 200; i++) {
      setMotors((i / 15) % 2 == 0 ? 60 : -60, (i / 15) % 2 == 0 ? -60 : 60);
      qtr.calibrate();
    }
    tocVlavo(60, 150);
    zastavsa();
    digitalWrite(LED_BUILTIN, LOW);
    stav = 2;
    Serial.println("Kalibrácia hotová. Start!");
    delay(1000);
    casstartu = millis();

  } else if (stav == 2) {
    if (digitalRead(Tlak) == HIGH && (millis() - casstartu > (long)IgnoreTime * 1000)) {
      vyhybajSaTehle();
    }
    pidStep();

  } else if (stav == 3) {
    zastavsa();
    static unsigned long poslednyBlik = 0;
    if (millis() - poslednyBlik > 500) {
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      poslednyBlik = millis();
    }
  }
}
