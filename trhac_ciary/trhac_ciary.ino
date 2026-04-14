#include <QTRSensors.h>

QTRSensors qtr;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount] = { 0 };

// --- STAVOVÉ PREMENNÉ ---
int stav = 1;                // 1=calibrate, 2=line follow, 3=stop
bool calibration_done = false;
bool dekaRezim = false;      // Signál pre spustenie 2s tlačenia deky
long casstartu = 0;

// --- NASTAVENIA SLEDOVANIA ČIARY ---
int baseSpeed = 50;
float kp = 1.4;
int rych_tocenia = 110;
int tehla = 85;              // Rýchlosť otáčania pri kontrole prekážky
int last_position = 3500;

//--------------------------------------------------------------------
// PINY
//--------------------------------------------------------------------
// MX1508 Motor Driver
#define LEFT_FWD  6
#define LEFT_BWD  5
#define RIGHT_FWD 10
#define RIGHT_BWD 9

// HY-SRF05 Ultrazvuk
#define TRIG_PIN 11
#define ECHO_PIN A0

//--------------------------------------------------------------------
// FUNKCIA: Meranie vzdialenosti (v cm)
//--------------------------------------------------------------------
long zmerajVzdialenost() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Timeout 4000us (cca 68 cm) zabráni zaseknutiu programu pri oslepení
  long duration = pulseIn(ECHO_PIN, HIGH, 4000);
  if (duration == 0) return 999; // Ak nič nevidí, vráti veľkú hodnotu
  long vzdialenost = duration * 0.034 / 2; 
  return vzdialenost;
}

//--------------------------------------------------------------------
// FUNKCIE PRE MOTORY
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
  setMotors(0, 0);
}

//--------------------------------------------------------------------
// FUNKCIA: Detekcia prekážky (Tehla vs. Deka)
//--------------------------------------------------------------------
void detekcia(int a) {
  // 1. Manéver: Pozri sa doprava
  setMotors(a, -a); 
  delay(250);       
  zastavsa();
  delay(150); // Krátka pauza na ustálenie senzora

  long vzdialenostT = zmerajVzdialenost();

  // Ak po otočení vidím prekážku -> DEKA (je široká)
  if (vzdialenostT < 60) { 
    Serial.println("Je to široké -> DEKA");
    // Toč sa späť doľava, aby si sa vyrovnal na čiaru
    setMotors(-a, a); 
    delay(230);
    zastavsa();
    
    // Aktivujeme 2-sekundový režim tlačenia deky
    dekaRezim = true; 
  } 
  // Ak po otočení nevidím nič -> TEHLA (je úzka a zmizla zo zorného pola)
  else {
    Serial.println("Zmizlo to -> TEHLA");
    zastavsa();
    while (true) {
      setMotors(0, 0); // Konečná zastávka - koniec programu
    }
  }
}

//--------------------------------------------------------------------
// SETUP
//--------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

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
  delay(3000); // Čas na položenie robota na čiaru pred kalibráciou
}

//--------------------------------------------------------------------
// MAIN LOOP
//--------------------------------------------------------------------
void loop() {
  
  // --- REŽIM 0: Špeciálny 2-sekundový režim pre DEKU ---
  if (dekaRezim == true) {
    Serial.println("Start 2s tlacenia deky po ciare...");
    unsigned long zaciatok = millis();
    
    // Tento cyklus pobeží presne 2 sekundy, robot stále reguluje PID
    while (millis() - zaciatok < 2000) {
      uint16_t position = qtr.readLineBlack(sensorValues);
      int error = (int)position - 3500;
      int correction = error * kp / 100;
      
      int leftSpeed  = baseSpeed - correction; 
      int rightSpeed = baseSpeed + correction; 
      
      setMotors(leftSpeed, rightSpeed);
      last_position = position;
    }
    
    // Po 2 sekundách režim vypneme a program plynule pokračuje do STAVU 2
    dekaRezim = false; 
    Serial.println("Deka prejduta.");
  }

  // --- STAV 1: Kalibrácia senzorov (spustí sa iba raz na začiatku) ---
  else if (stav == 1) {
    Serial.println("Kalibracia zacina...");
    digitalWrite(LED_BUILTIN, HIGH);
    
    for (uint16_t i = 0; i < 400; i++) {
      if ((i / 20) % 2 == 0) {
        setMotors(50, -50); 
      } else {
        setMotors(-50, 50);
      }
      qtr.calibrate();
      delay(5); // Nutná pauza, aby mal robot čas sa fyzicky hýbať
    }
    
    zastavsa();
    digitalWrite(LED_BUILTIN, LOW);
    calibration_done = true;
    stav = 2; // Prepnutie do režimu sledovania čiary
    Serial.println("Kalibracia dokoncena.");
    delay(2000);  
    casstartu = millis();
  }
  
  // --- STAV 2: Sledovanie čiary a hľadanie prekážky ---
  else if (stav == 2) {
    long vzdialenost = zmerajVzdialenost();
    
    // Ak uvidíme prekážku tesne pred sebou (menej ako 10 cm)
    if (vzdialenost > 0 && vzdialenost < 10) {
      detekcia(tehla); // Spustí kontrolu Tehla vs Deka
    }

    uint16_t position = qtr.readLineBlack(sensorValues);
    
    // Logika pri strate čiary (ostré zákruty)
    if (last_position < 1000 && position == 0){
      setMotors(rych_tocenia, 0);
    }
    else if(last_position > 6000 && position == 7000){
      setMotors(0, rych_tocenia);
    }
    // Stratenie čiary na križovatkách alebo 90 stupňoch
    else if ((last_position < 6000 && last_position > 1000) && (position == 0 || position == 7000)){
      last_position = position;
      while ((position == 0 || position == 7000) && (last_position == 0 || last_position == 7000)){
        setMotors(baseSpeed, baseSpeed);
        position = qtr.readLineBlack(sensorValues);
        last_position = position;
      }
    }
    // Štandardné PID sledovanie (ak sme na čiare)
    else {
      int error = (int)position - 3500;
      int correction = error * kp / 100;
      
      int leftSpeed  = baseSpeed - correction; 
      int rightSpeed = baseSpeed + correction; 
      
      setMotors(leftSpeed, rightSpeed);
      last_position = position;
    }
  }
  
  // --- STAV 3: NÚDZOVÉ ZASTAVENIE ---
  else if (stav == 3) {
    zastavsa();
  }
}