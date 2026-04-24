#include <QTRSensors.h>
#include <Wire.h>
#include <VL53L1X.h>

QTRSensors qtr;
VL53L1X tof;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount] = { 0 };

int stav = 1;
bool calibration_done = false;
long casstartu = 0;

// Line follower
int baseSpeed = 220;
float kp = 0.45;
int rych_tocenia = 255;
int last_position = 3500;

// VL53L1X
#define TOF_OBSTACLE_MM  200
bool tof_ok = false;

// Pins MX1508
#define LEFT_FWD  6
#define LEFT_BWD  5
#define RIGHT_FWD 10
#define RIGHT_BWD 9

//--------------------------------------------------------------------
// MOTORS
//--------------------------------------------------------------------

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
  analogWrite(LEFT_FWD, 0);
  analogWrite(LEFT_BWD, 0);
  analogWrite(RIGHT_FWD, 0);
  analogWrite(RIGHT_BWD, 0);
}

//--------------------------------------------------------------------
// VL53L1X
//--------------------------------------------------------------------

bool isObstacleDetected() {
  if (!tof_ok) return false;
  if (tof.dataReady()) {
    uint16_t dist = tof.read();
    Serial.print("TOF dist: ");
    Serial.println(dist);
    if (dist > 0 && dist < TOF_OBSTACLE_MM) {
      return true;
    }
  }
  return false;
}

//--------------------------------------------------------------------
// Setup
//--------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  Serial.println("Start MCU");

  Wire.begin();
  Wire.setClock(400000);
  tof.setTimeout(500);
  if (tof.init()) {
    tof.setDistanceMode(VL53L1X::Long);
    tof.setMeasurementTimingBudget(20000);
    tof.startContinuous(20);
    tof_ok = true;
    Serial.println("VL53L1X OK");
  } else {
    Serial.println("VL53L1X NENAJDENY – pokracujem bez TOF");
  }

  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){ 15, 16, 17, 2, 3, 4, 7, 8 }, SensorCount);
  qtr.setEmitterPin(12);

  pinMode(LEFT_FWD, OUTPUT);
  pinMode(LEFT_BWD, OUTPUT);
  pinMode(RIGHT_FWD, OUTPUT);
  pinMode(RIGHT_BWD, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  digitalWrite(LEFT_FWD, LOW);
  digitalWrite(LEFT_BWD, LOW);
  digitalWrite(RIGHT_FWD, LOW);
  digitalWrite(RIGHT_BWD, LOW);

  delay(1000);
  Serial.println("Configuration done");

  stav = 1;
  Serial.println("Auto-calibration in 3 sec - prepare to move robot!");
  delay(3000);
}

//--------------------------------------------------------------------
// Main loop
//--------------------------------------------------------------------

void loop() {

  if (stav == 2 && isObstacleDetected()) {
    Serial.println("PREKAZKA! Zastavujem.");
    stav = 3;
  }

  if (stav == 0) {
    stav = 1;
    Serial.println("Prechadzam na kalibraciu");

  } else if (stav == 1) {
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.println("Kmitava kalibracia (zmena kazdych 50 cyklov)...");

    for (uint16_t i = 0; i < 500; i++) {
      if ((i / 10) % 2 == 0) {
        setMotors(50, -50);
      } else {
        setMotors(-50, 50);
      }
      qtr.calibrate();
      delay(4);
    }

    zastavsa();
    digitalWrite(LED_BUILTIN, LOW);

    Serial.print("Kalibracia hotova. Min hodnoty: ");
    for (uint8_t i = 0; i < SensorCount; i++) {
      Serial.print(qtr.calibrationOn.minimum[i]);
      Serial.print(' ');
    }
    Serial.println();

    calibration_done = true;
    stav = 2;

    Serial.println("Priprava na start...");
    delay(2000);
    casstartu = millis();

  } else if (stav == 2) {
    uint16_t position = qtr.readLineBlack(sensorValues);

    // Detect gap (dashed line)
    bool allLow = true;
    for (uint8_t i = 0; i < SensorCount; i++) {
      if (sensorValues[i] > 200) { allLow = false; break; }
    }

    if (allLow) {
      setMotors(baseSpeed, baseSpeed);
      return;
    }

    // Line lost on one side
    if (last_position < 1000 && position == 0) {
      setMotors(rych_tocenia, -rych_tocenia);
      return;
    }
    if (last_position > 6000 && position == 7000) {
      setMotors(-rych_tocenia, rych_tocenia);
      return;
    }

    // P control
    int error = (int)position - 3500;
    int correction = error * kp;

    int leftSpeed  = constrain(baseSpeed - correction, -rych_tocenia, rych_tocenia);
    int rightSpeed = constrain(baseSpeed + correction, -rych_tocenia, rych_tocenia);

    setMotors(leftSpeed, rightSpeed);
    last_position = position;

  } else if (stav == 3) {
    zastavsa();
  }
}
