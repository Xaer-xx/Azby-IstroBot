#include <Wire.h>
#include <QTRSensors.h>
#include <VL53L1X.h>

QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount] = { 0 };

VL53L1X sensor;
uint16_t lastDistance = 0;

#define LEFT_FWD  6
#define LEFT_BWD  5
#define RIGHT_FWD 10
#define RIGHT_BWD 9

int stav = 1;
bool calibration_done = false;
bool dekaRezim = false;
long casstartu = 0;
int last_position = 3500;
int last_error = 0;

// ======================================================
// ==================== DOLADENIE =======================
// ======================================================
int IgnoreTime         = 2;
int atencion           = 15;

int baseSpeed          = 240; 
float kp               = 0.1; 
float kd               = 0.7; 

int rych_tocenia       = 255;
int tehla              = 120;

int sharpTurnThreshold = 200;
int minTurnSpeed       = 40;

int T_TURN_90   = 280;
int T_OBIST_BOK = 400;
int T_OBIST_DLZ = 600;

// --- NOVÉ: Čas prejazdu cez farbu (P7) a prerušenú čiaru (P2) ---
int limitSlepejJazdy  = 800; 

// ======================================================
// ============= MERANIE VZDIALENOSTI ===================
// ======================================================
long zmerajVzdialenost() {
  if (sensor.dataReady()) {
    lastDistance = sensor.read(false);
  }
  return lastDistance / 10;
}

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
// ======= ADAPTIVNÁ RÝCHLOSŤ ===========================
// ======================================================
int getAdaptiveSpeed(int error) {
  int absError = abs(error);
  if (absError <= sharpTurnThreshold) return baseSpeed;
  return constrain(
    map(absError, sharpTurnThreshold, 3500, baseSpeed, minTurnSpeed),
    minTurnSpeed, baseSpeed
  );
}

// ======================================================
// ========= PD REGULÁTOR - JADRO ROBOTA ================
// ======================================================
void pidStep() {
  uint16_t position = qtr.readLineBlack(sensorValues);
  int error = (int)position - 3500;

  // -------- STRATA ČIARY --------
  if (position == 0 || position == 7000) {

    if (last_position < 1000 && position == 0) {
      // Ľavá strana - tocíme doľava kým nenájdeme čiaru
      do {
        setMotors(-rych_tocenia, rych_tocenia);
        position = qtr.readLineBlack(sensorValues);
      } while (position == 0);

    } else if (last_position > 6000 && position == 7000) {
      // Pravá strana - točíme doprava
      do {
        setMotors(rych_tocenia, -rych_tocenia);
        position = qtr.readLineBlack(sensorValues);
      } while (position == 7000);

    } else {
      // Stred - PRERUŠENÁ ČIARA (P2), FARBA (P7) alebo návrat z tehly
      unsigned long t = millis();
      while ((position == 0 || position == 7000) && millis() - t < limitSlepejJazdy) {
        setMotors(baseSpeed, baseSpeed);
        position = qtr.readLineBlack(sensorValues);
      }
    }

    last_position = position;
    last_error = 0;   
    return;
  }

  // -------- PD VÝPOČET --------
  int correction = (int)(kp * error) + (int)(kd * (error - last_error));
  int speed = getAdaptiveSpeed(error);

  setMotors(speed - correction, speed + correction);

  last_position = position;
  last_error = error;   
}

// ======================================================
// ============= VYHYBANIE SA TEHLE =====================
// ======================================================
void vyhybajSaTehle() {
  Serial.println("TEHLA - obchadzanie");
  zastavsa(); delay(100);

  tocVpravo(tehla, T_TURN_90); delay(100);
  idDopredu(baseSpeed, T_OBIST_BOK); delay(100);
  tocVlavo(tehla, T_TURN_90); delay(90);
  idDopredu(baseSpeed, T_OBIST_DLZ); delay(100);
  tocVlavo(tehla, T_TURN_90); delay(80);

  unsigned long timeout = millis();
  while (millis() - timeout < 3000) {
    setMotors(baseSpeed, baseSpeed);
    uint16_t pos = qtr.readLineBlack(sensorValues);
    if (pos > 500 && pos < 6500) { zastavsa(); break; }
  }
  delay(100);

  tocVpravo(tehla, T_TURN_90); delay(100);
  last_position = 3500;
  last_error = 0;
}

// ======================================================
// ========= DETEKCIA TEHLA vs DEKA =====================
// ======================================================
void detekcia(int a) {
  zastavsa(); delay(100);
  tocVpravo(a, 280); delay(50);

  long d = zmerajVzdialenost();

  if (d < 60 && d > 0) {
    Serial.println("DEKA");
    tocVlavo(a, 280); delay(100);
    dekaRezim = true;
  } else {
    Serial.println("TEHLA");
    tocVlavo(a, 280); delay(100);
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

  sensor.setTimeout(500);
  if (!sensor.init()) {
    Serial.println("VL53L1X nenajdeny!");
    while (1);
  }
  sensor.setDistanceMode(VL53L1X::Long);
  sensor.setMeasurementTimingBudget(20000);
  sensor.startContinuous(10);

  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){ 15, 16, 17, 2, 3, 4, 7, 8 }, SensorCount);
  qtr.setEmitterPin(12);

  pinMode(LEFT_FWD, OUTPUT); pinMode(LEFT_BWD, OUTPUT);
  pinMode(RIGHT_FWD, OUTPUT); pinMode(RIGHT_BWD, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  zastavsa();
  delay(1000);
  stav = 1;
  delay(3000);
}

// ======================================================
// ====================== LOOP ==========================
// ======================================================
void loop() {

  if (dekaRezim) {
    unsigned long zaciatok = millis();
    while (millis() - zaciatok < (IgnoreTime * 1000)) {
      pidStep();   
    }
    dekaRezim = false;

  } else if (stav == 1) {
    Serial.println("Kalibracia...");
    digitalWrite(LED_BUILTIN, HIGH);
    for (uint16_t i = 0; i < 400; i++) {
      setMotors((i / 20) % 2 == 0 ? 50 : -50,
                (i / 20) % 2 == 0 ? -50 : 50);
      qtr.calibrate();
      delay(5);
    }
    zastavsa();
    digitalWrite(LED_BUILTIN, LOW);
    calibration_done = true;
    stav = 2;
    last_error = 0;
    Serial.println("Kalibracia hotova.");
    delay(2000);
    casstartu = millis();

  } else if (stav == 2) {
    long vzd = zmerajVzdialenost();
    if (vzd > 0 && vzd < atencion) {
      detekcia(tehla);
      return;
    }
    pidStep();

  } else if (stav == 3) {
    zastavsa();
  }
}