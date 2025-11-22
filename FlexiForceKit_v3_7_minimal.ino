/*
  FlexiForceKit v3.7 - Minimal
  - NÃO mexe em RsSelectA, RsSelectB nem SensorDriveOnOff.
  - Lê SensorPin = A2, LoadCellPin = A6.
  - Envia para GUI: pos,0,vd,lc
  - Mostra DBG: A2=... no Serial Monitor.
*/

#include <Servo.h>

// ------------------ PINOS ------------------
#define ServoPin          10
#define Green_LED         2
#define Yellow_LED        5
#define Red_LED           A4
#define Mod1LED           3
#define Mod2LED           6
#define SensorDriveOnOff  7     // NÃO USADO AQUI
#define RsSelectA         9     // NÃO USADO AQUI
#define RsSelectB         8     // NÃO USADO AQUI

#define LoadCellPin       A6
#define SensorPin         A2    // Sinal do FlexiForce (igual diagnóstico)

// ------------------ POSIÇÕES MECÂNICAS ------------------
const int Start_Position       = 1500;
const int PreLoadingPosition   = 1340;
const int DriftPreloadPosition = 1328;

const int Num_Steps = 5;
int Step_Position[Num_Steps] = {1338, 1323, 1308, 1293, 1278};

const int Min_Servo_us = 1200;
const int Max_Servo_us = 1600;

// ------------------ STREAM DE DADOS ------------------
unsigned long lastSample = 0;
const unsigned long sampleIntervalMs = 50;

// ------------------ DRIFT ------------------
const int Drift_Num_Samples = 60;
unsigned long driftIntervalMs = 1000;

// ------------------ OBJETOS ------------------
Servo linearServo;

// ------------------ ESTADO DO TESTE ------------------
enum TestMode {
  TEST_NONE = 0,
  TEST_LINEARITY = 1,
  TEST_LIN_HYST = 2,
  TEST_DRIFT = 3,
  TEST_REPEAT = 4
};

TestMode currentTest = TEST_NONE;
int currentStep = 0;
int repeatCount = 0;
int maxRepeats  = 3;
unsigned long stepStartMillis = 0;
unsigned long testHoldMs = 2000;

// ------------------ ESTADO DO SERVO ------------------
int currentServoPos_us = Start_Position;

// ------------------ AUXILIARES ------------------
int clampServo(int us) {
  if (us < Min_Servo_us) us = Min_Servo_us;
  if (us > Max_Servo_us) us = Max_Servo_us;
  return us;
}

void goToPosition(int us) {
  us = clampServo(us);
  currentServoPos_us = us;
  linearServo.writeMicroseconds(us);
}

int readAnalogFiltered(uint8_t pin) {
  int v1 = analogRead(pin);
  int v2 = analogRead(pin);
  int v3 = analogRead(pin);
  // mediana simples
  if (v1 > v2) { int t = v1; v1 = v2; v2 = t; }
  if (v2 > v3) { int t = v2; v2 = v3; v3 = t; }
  if (v1 > v2) { int t = v1; v1 = v2; v2 = t; }
  return v2;
}

// ------------------ SERIAL / COMANDOS ------------------
String serialLine;

void handleSerialCommand(const String &cmd) {
  if (cmd.length() == 0) return;

  if (cmd.startsWith("MOVE:")) {
    int us = cmd.substring(5).toInt();
    currentTest = TEST_NONE;
    goToPosition(us);
    Serial.println("OK MOVE");
    return;
  }

  if (cmd.equals("GOTO:HOME")) {
    currentTest = TEST_NONE;
    goToPosition(PreLoadingPosition);
    Serial.println("OK GOTO HOME");
    return;
  }

  if (cmd.equals("GOTO:ZERO")) {
    currentTest = TEST_NONE;
    goToPosition(Start_Position);
    Serial.println("OK GOTO ZERO");
    return;
  }

  if (cmd.equals("STOP")) {
    currentTest = TEST_NONE;
    Serial.println("OK STOP");
    return;
  }

  if (cmd.startsWith("SET:REPEAT:")) {
    int n = cmd.substring(12).toInt();
    if (n < 1) n = 1;
    if (n > 99) n = 99;
    maxRepeats = n;
    Serial.print("OK SET REPEAT ");
    Serial.println(maxRepeats);
    return;
  }

  if (cmd.equals("TEST:LINEARITY")) {
    currentTest = TEST_LINEARITY;
    currentStep = 0;
    stepStartMillis = millis();
    goToPosition(Step_Position[0]);
    Serial.println("OK TEST LINEARITY");
    return;
  }

  if (cmd.equals("TEST:LIN_HYST")) {
    currentTest = TEST_LIN_HYST;
    currentStep = 0;
    stepStartMillis = millis();
    goToPosition(Step_Position[0]);
    Serial.println("OK TEST LIN_HYST");
    return;
  }

  if (cmd.equals("TEST:DRIFT")) {
    currentTest = TEST_DRIFT;
    stepStartMillis = millis();
    goToPosition(DriftPreloadPosition);
    Serial.println("OK TEST DRIFT");
    return;
  }

  if (cmd.equals("TEST:REPEAT")) {
    currentTest = TEST_REPEAT;
    repeatCount = 0;
    currentStep = 0;
    stepStartMillis = millis();
    goToPosition(Step_Position[0]);
    Serial.println("OK TEST REPEAT");
    return;
  }

  Serial.println("ERR CMD");
}

void pollSerial() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n') {
      serialLine.trim();
      handleSerialCommand(serialLine);
      serialLine = "";
    } else if (c != '\r') {
      serialLine += c;
    }
  }
}

// ------------------ LÓGICA DOS TESTES ------------------
void updateTestState() {
  unsigned long now = millis();

  switch (currentTest) {
    case TEST_NONE:
      break;

    case TEST_LINEARITY:
      if (now - stepStartMillis >= testHoldMs) {
        currentStep++;
        if (currentStep >= Num_Steps) {
          currentTest = TEST_NONE;
          Serial.println("DONE LINEARITY");
        } else {
          goToPosition(Step_Position[currentStep]);
          stepStartMillis = now;
        }
      }
      break;

    case TEST_LIN_HYST: {
      static bool descending = false;
      if (now - stepStartMillis >= testHoldMs) {
        stepStartMillis = now;

        if (!descending) {
          currentStep++;
          if (currentStep >= Num_Steps) {
            descending = true;
            currentStep = Num_Steps - 2;
          }
        } else {
          currentStep--;
          if (currentStep < 0) {
            currentTest = TEST_NONE;
            descending = false;
            currentStep = 0;
            Serial.println("DONE LIN_HYST");
            break;
          }
        }

        if (currentTest != TEST_NONE) {
          goToPosition(Step_Position[currentStep]);
        }
      }
      break;
    }

    case TEST_DRIFT:
      if (now - stepStartMillis >= (unsigned long)Drift_Num_Samples * driftIntervalMs) {
        currentTest = TEST_NONE;
        Serial.println("DONE DRIFT");
      }
      break;

    case TEST_REPEAT:
      if (now - stepStartMillis >= testHoldMs) {
        currentStep++;

        if (currentStep >= Num_Steps) {
          currentStep = 0;
          repeatCount++;

          if (repeatCount >= maxRepeats) {
            currentTest = TEST_NONE;
            Serial.println("DONE REPEAT");
          }
        }

        if (currentTest != TEST_NONE) {
          goToPosition(Step_Position[currentStep]);
          stepStartMillis = now;
        }
      }
      break;
  }
}

// ------------------ STREAM DE DADOS ------------------
void streamDataIfNeeded() {
  unsigned long now = millis();
  if (now - lastSample < sampleIntervalMs) return;
  lastSample = now;

  int pos    = currentServoPos_us;
  int vd_raw = readAnalogFiltered(SensorPin);
  int lc     = readAnalogFiltered(LoadCellPin);

  // Debug: ver valor real em A2
  Serial.print("DBG: A2=");
  Serial.println(vd_raw);

  // Linha para a GUI: pos,op,vd,lc
  Serial.print(pos);
  Serial.print(',');
  Serial.print(0);        // opamp não usado
  Serial.print(',');
  Serial.print(vd_raw);   // voltdiv (FlexiForce)
  Serial.print(',');
  Serial.println(lc);     // loadcell
}

// ------------------ SETUP ------------------
void setup() {
  Serial.begin(9600);
  delay(400);

  linearServo.attach(ServoPin);
  goToPosition(Start_Position);

  pinMode(Green_LED, OUTPUT);
  pinMode(Yellow_LED, OUTPUT);
  pinMode(Red_LED, OUTPUT);
  pinMode(Mod1LED, OUTPUT);
  pinMode(Mod2LED, OUTPUT);

  digitalWrite(Green_LED, LOW);
  digitalWrite(Yellow_LED, LOW);
  digitalWrite(Red_LED, LOW);
  digitalWrite(Mod1LED, LOW);
  digitalWrite(Mod2LED, LOW);

  // NÃO configuramos SensorDriveOnOff, RsSelectA, RsSelectB:
  // eles ficam no estado padrão (INPUT) igual no diagnóstico.

  Serial.println("FlexiForce firmware v3.7 (minimal, SensorPin=A2, sem controle de RsSelect).");
}

// ------------------ LOOP ------------------
void loop() {
  pollSerial();
  updateTestState();
  streamDataIfNeeded();
}
