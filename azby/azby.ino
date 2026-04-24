#include <Wire.h>
#include <QTRSensors.h>
#include <VL53L1X.h>

// ======================================================
// =================== QTR SENZORY ======================
// ======================================================
QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount] = { 0 };

// ======================================================
// ================== VL53L1X SENZOR ====================
// ======================================================
const uint8_t SENSOR_COUNT = 1;
VL53L1X sensors[SENSOR_COUNT];
uint16_t sensorDistance[SENSOR_COUNT];
uint16_t lastDistances[SENSOR_COUNT] = { 0 };

// ======================================================
// =================== MOTOR PINY =======================
// ======================================================
#define LEFT_FWD  6
#define LEFT_BWD  5
#define RIGHT_FWD 10
#define RIGHT_BWD 9

// ======================================================
// ================ STAVOVÉ PREMENNÉ ====================
// ======================================================
int stav = 1;
bool calibration_done = false;
bool dekaRezim = false;
long casstartu = 0;
int last_position = 3500;

// ======================================================
// ==================== DOLADENIE =======================
// ======================================================
int IgnoreTime    = 2;    // sekundy tlacenia deky
int atencion      = 15;   // cm - prah detekcie prekazky

int baseSpeed = 220;
float kp = 0.45;
int rych_tocenia = 255;
int last_position = 3500;

// ======================================================
// ============= SPOMALOVANIE V OSTRYCH ZATACKACH =======
// ======================================================
// Prah chyby, od ktoreho sa zacina spomalovat (0-3500).
// Mensie cislo = zacne spomalovat skor (citlivejsie).
// Vacsie cislo = spomaluje len v naozaj ostrych zatackach.
int sharpTurnThreshold = 1500;

// Minimalná rychlost pri ostrych zatackach (0-baseSpeed).
// Zvys ak kolieska pri otacani klzu alebo motor stoji.
int minTurnSpeed = 60;

// ======================================================
// ===== DOLADENIE MANÉVRU OBCHÁDZANIA (v ms) ===========
// ======================================================
int T_TURN_90   = 280;
int T_OBIST_BOK = 400;
int T_OBIST_DLZ = 600;

// ======================================================
// ============= MERANIE VZDIALENOSTI (cm) ==============
// ======================================================
long zmerajVzdialenost() {
  if (sensors[0].dataReady()) {
    lastDistances[0] = sensors[0].read(false);
  }
  sensorDistance[0] = lastDistances[0];
  return sensorDistance[0] / 10;
}

// ======================================================
// ==================== MOTORY ==========================
// ======================================================
void setMotors(int left, int right) {
  if (left >= 0) {
    analogWrite(LEFT_FWD, constrain(left, 0, 255));
    analogWrite(LEFT_BWD, 0);
  } else {
    analogWrite(LEFT_FWD, 0);
    analogWrite(LEFT_BWD, constrain(-left, 0, 255));
  }
  if (right >= 0) {
    analogWrite(RIGHT_FWD, constrain(right, 0, 255));
    analogWrite(RIGHT_BWD, 0);
  } else {
    analogWrite(RIGHT_FWD, 0);
    analogWrite(RIGHT_BWD, constrain(-right, 0, 255));
  }
}

void zastavsa() {
  setMotors(0, 0);
}

void tocVpravo(int a, int ms) {
  setMotors(a, -a);
  delay(ms);
  zastavsa();
}

void tocVlavo(int a, int ms) {
  setMotors(-a, a);
  delay(ms);
  zastavsa();
}

void idDopredu(int rychlost, int ms) {
  setMotors(rychlost, rychlost);
  delay(ms);
  zastavsa();
}

// ======================================================
// ======= VYPOCET RYCHLOSTI PODLA OSTROSTI ZATACKY =====
// ======================================================
// Vracia rychlost znizenu podla toho, ako ostro sa zataca.
// Pri malej chybe (rovinka) = plna rychlost (baseSpeed).
// Pri velkej chybe (ostra zatacka) = minTurnSpeed.
int getAdaptiveSpeed(int error) {
  int absError = abs(error);
  if (absError <= sharpTurnThreshold) {
    // Rovinka alebo mirna zatacka -> plna rychlost
    return baseSpeed;
  }
  // Ostra zatacka: linearne spomalenie od baseSpeed po minTurnSpeed
  int slowdown = map(absError, sharpTurnThreshold, 3500, baseSpeed, minTurnSpeed);
  return constrain(slowdown, minTurnSpeed, baseSpeed);
}

// ======================================================
// ============= VYHYBANIE SA TEHLE =====================
// ======================================================
void vyhybajSaTehle() {
  Serial.println("TEHLA - zacina obchadzanie");
  zastavsa();
  delay(100);

  Serial.println("Krok 1: tocim vpravo");
  tocVpravo(tehla, T_TURN_90);
  delay(100);

  Serial.println("Krok 2: idem bokom");
  idDopredu(baseSpeed, T_OBIST_BOK);
  delay(100);

  Serial.println("Krok 3: tocim vlavo");
  tocVlavo(tehla, T_TURN_90);
  delay(100);

  Serial.println("Krok 4: idem pozdlz tehly");
  idDopredu(baseSpeed, T_OBIST_DLZ);
  delay(100);

  Serial.println("Krok 5: tocim vlavo k ceste");
  tocVlavo(tehla, T_TURN_90);
  delay(100);

  Serial.println("Krok 6: hladam ciaru...");
  unsigned long timeout = millis();
  while (millis() - timeout < 3000) {
    setMotors(baseSpeed, baseSpeed);
    uint16_t pos = qtr.readLineBlack(sensorValues);
    if (pos > 500 && pos < 6500) {
      zastavsa();
      Serial.println("Ciara najdena!");
      break;
    }
  }
  delay(100);

  Serial.println("Krok 7: vyrovnavam smer");
  tocVpravo(tehla, T_TURN_90);
  delay(100);

  Serial.println("TEHLA - obchadzanie dokoncene");
  last_position = 3500;
}

// ======================================================
// ========= DETEKCIA (TEHLA vs DEKA) ===================
// ======================================================
void detekcia(int a) {
  zastavsa();
  delay(100);

  tocVpravo(a, 250);
  delay(50);

  long vzdialenostT = zmerajVzdialenost();

  if (vzdialenostT < 60) {
    Serial.println("Je to siroke -> DEKA");
    tocVlavo(a, 250);
    delay(100);
    dekaRezim = true;
  } else {
    Serial.println("Zmizlo to -> TEHLA");
    tocVlavo(a, 250);
    delay(100);
    vyhybajSaTehle();
  }
}

// ======================================================
// ====================== SETUP =========================
// ======================================================
void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);

  sensors[0].setTimeout(500);
  if (!sensors[0].init()) {
    Serial.println("Failed to detect VL53L1X sensor");
    while (1);
  }
  sensors[0].setDistanceMode(VL53L1X::Long);
  sensors[0].setMeasurementTimingBudget(20000);
  sensors[0].startContinuous(10);

  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){ 15, 16, 17, 2, 3, 4, 7, 8 }, SensorCount);
  qtr.setEmitterPin(12);

  pinMode(LEFT_FWD, OUTPUT);
  pinMode(LEFT_BWD, OUTPUT);
  pinMode(RIGHT_FWD, OUTPUT);
  pinMode(RIGHT_BWD, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  zastavsa();
  delay(1000);
  Serial.println("Configuration done");

  stav = 1;
  delay(3000);
}

// ======================================================
// ====================== LOOP ==========================
// ======================================================
void loop() {

  // --- REZIM 0: Tlacenie deky ---
  if (dekaRezim == true) {
    Serial.println("Start tlacenia deky...");
    unsigned long zaciatok = millis();

    while (millis() - zaciatok < (IgnoreTime * 1000)) {
      uint16_t position = qtr.readLineBlack(sensorValues);
      int error = (int)position - 3500;
      int correction = error * kp / 100;
      // Spomalenie funguje aj pri tlaceni deky
      int speed = getAdaptiveSpeed(error);
      setMotors(speed - correction, speed + correction);
      last_position = position;
    }

    dekaRezim = false;
    Serial.println("Deka prejduta.");
  }

  // --- STAV 1: Kalibracia ---
  else if (stav == 1) {
    Serial.println("Kalibracia zacina...");
    digitalWrite(LED_BUILTIN, HIGH);

    for (uint16_t i = 0; i < 400; i++) {
      if ((i / 20) % 2 == 0) setMotors(50, -50);
      else                    setMotors(-50, 50);
      qtr.calibrate();
      delay(5);
    }

    zastavsa();
    digitalWrite(LED_BUILTIN, LOW);
    calibration_done = true;
    stav = 2;
    Serial.println("Kalibracia dokoncena.");
    delay(2000);
    casstartu = millis();
  }

  // --- STAV 2: Sledovanie ciary + detekcia prekazky ---
 } else if (stav == 2) {
    uint16_t position = qtr.readLineBlack(sensorValues);

    bool allLow = true;
    for (uint8_t i = 0; i < SensorCount; i++) {
        if (sensorValues[i] > 200) { allLow = false; break; }
    }

    if (allLow) {
        setMotors(baseSpeed, baseSpeed);
        return;
    }

    if (last_position < 1000 && position == 0) {
        setMotors(rych_tocenia, -rych_tocenia);
        return;
    }
    if (last_position > 6000 && position == 7000) {
        setMotors(-rych_tocenia, rych_tocenia);
        return;
    }

    int error = (int)position - 3500;
    int correction = error * kp;

    int leftSpeed  = constrain(baseSpeed - correction, -rych_tocenia, rych_tocenia);
    int rightSpeed = constrain(baseSpeed + correction, -rych_tocenia, rych_tocenia);

    setMotors(leftSpeed, rightSpeed);
    last_position = position;

    // -------------------------------------------------------
    // NORMALNE PID SLEDOVANIE S ADAPTIVNOU RYCHLOSTOU
    // -------------------------------------------------------
    } else {
      int error = (int)position - 3500;
      int correction = error * kp / 100;

      // Ziskaj rychlost podla ostrosti zatacky
      int speed = getAdaptiveSpeed(error);

      setMotors(speed - correction, speed + correction);
      last_position = position;
    }
  }

  // --- STAV 3: NUDZOVE ZASTAVENIE ---
  else if (stav == 3) {
    zastavsa();
  }
}
