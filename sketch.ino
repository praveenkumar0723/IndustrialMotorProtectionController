
#include <Arduino.h>

// ==================================================
// INDUSTRIAL MOTOR PROTECTION CONTROLLER
// Board: STM32 Nucleo-C031C6 (Wokwi)
// ==================================================

// Inputs
#define PIN_SPD   A0
#define PIN_CUR   A1
#define PIN_TMP   A2
#define PIN_REF   A3
#define PIN_START D2
#define PIN_STOP  D4

// Outputs
#define PIN_PWM   D5
#define PIN_EN    D6
#define PIN_FLT   D7
#define PIN_BUZ   D8
#define PIN_LRUN  D9
#define PIN_LIDLE D10

// Sensor scaling
#define MAX_RPM   3000.0f
#define MAX_AMPS  10.0f
#define MAX_TEMP  120.0f

// Protection thresholds
#define OC_LIMIT      8.0f
#define OT_LIMIT      80.0f
#define OT_CLEAR      70.0f
#define OS_ABS_LIMIT  3300.0f

#define STALL_AMPS    5.0f
#define STALL_RPM     100.0f
#define STALL_DUTY    20.0f

// Detection delays
#define OC_MS  100UL
#define OT_MS  500UL
#define OS_MS  200UL
#define ST_MS  500UL

// Buzzer: 300 ms ON, 300 ms OFF
#define BUZZER_ON_MS   300UL
#define BUZZER_OFF_MS  300UL
#define BUZZER_FREQ    2000

enum State { IDLE, RUN, FAULT };
enum Fault { NONE, OVERCURRENT, OVERHEAT, OVERSPEED, STALL };

const char* stateName[] = {"IDLE", "RUN", "FAULT"};
const char* faultName[] = {
  "NONE", "OVERCURRENT", "OVERHEAT", "OVERSPEED", "STALL"
};

State state = IDLE;
Fault fault = NONE;

// Sensor readings
float rpm = 0.0f;
float amps = 0.0f;
float tempC = 0.0f;
float refRpm = 0.0f;

// Controller
float duty = 0.0f;
float integ = 0.0f;
float target = 0.0f;

// Protection timers
unsigned long tOC = 0;
unsigned long tOT = 0;
unsigned long tOS = 0;
unsigned long tST = 0;

// Main timing
unsigned long lastCtl = 0;
unsigned long lastPrint = 0;

// Buzzer state
bool buzzerOn = false;
unsigned long buzzerChangedAt = 0;

// --------------------------------------------------
// SENSOR READING
// --------------------------------------------------
float avg(int pin) {
  long sum = 0;

  for (int i = 0; i < 4; i++) {
    sum += analogRead(pin);
  }

  return sum / 4.0f;
}

void readSensors() {
  rpm    = avg(PIN_SPD) * MAX_RPM / 1023.0f;
  amps   = avg(PIN_CUR) * MAX_AMPS / 1023.0f;
  tempC  = avg(PIN_TMP) * MAX_TEMP / 1023.0f;
  refRpm = avg(PIN_REF) * MAX_RPM / 1023.0f;
}

// --------------------------------------------------
// FAULT TIMERS
// --------------------------------------------------
void resetTimers() {
  unsigned long now = millis();
  tOC = tOT = tOS = tST = now;
}

bool persist(bool condition, unsigned long &timer,
             unsigned long duration) {
  if (!condition) {
    timer = millis();
    return false;
  }

  return millis() - timer >= duration;
}

// --------------------------------------------------
// OVERSPEED CHECK
// Absolute limit always applies.
// Relative limit applies only with a valid reference.
// --------------------------------------------------
bool overspeedActive() {
  return rpm > OS_ABS_LIMIT ||
         (refRpm > 100.0f &&
          rpm > 1.1f * refRpm + 100.0f);
}

// --------------------------------------------------
// FAULT DETECTION
// --------------------------------------------------
Fault checkFaults() {
  if (persist(amps > OC_LIMIT, tOC, OC_MS)) {
    return OVERCURRENT;
  }

  if (persist(tempC > OT_LIMIT, tOT, OT_MS)) {
    return OVERHEAT;
  }

  if (persist(overspeedActive(), tOS, OS_MS)) {
    return OVERSPEED;
  }

  if (persist(
        state == RUN &&
        amps > STALL_AMPS &&
        rpm < STALL_RPM &&
        duty > STALL_DUTY,
        tST, ST_MS)) {
    return STALL;
  }

  return NONE;
}

// --------------------------------------------------
// CHECK WHETHER LATCHED FAULT IS SAFE TO RESET
// --------------------------------------------------
bool faultStillActive() {
  switch (fault) {
    case OVERCURRENT:
      return amps > OC_LIMIT;

    case OVERHEAT:
      return tempC > OT_CLEAR;

    case OVERSPEED:
      return overspeedActive();

    case STALL:
      return amps > STALL_AMPS &&
             rpm < STALL_RPM;

    default:
      return false;
  }
}

// --------------------------------------------------
// ENTER FAULT
// --------------------------------------------------
void enterFault(Fault newFault) {
  if (state == FAULT || newFault == NONE) {
    return;
  }

  fault = newFault;
  state = FAULT;

  duty = 0.0f;
  integ = 0.0f;
  target = 0.0f;

  // Start a fresh buzzer cycle immediately.
  noTone(PIN_BUZ);
  buzzerOn = false;
  buzzerChangedAt = millis() - BUZZER_OFF_MS;

  // Disable motor outputs immediately.
  analogWrite(PIN_PWM, 0);
  digitalWrite(PIN_EN, LOW);

  Serial.print("EVENT: FAULT -> ");
  Serial.println(faultName[fault]);
}

// --------------------------------------------------
// BUZZER: 300 ms BEEP, 300 ms SILENCE
// --------------------------------------------------
void updateBuzzer() {
  unsigned long now = millis();

  if (state != FAULT) {
    if (buzzerOn) {
      noTone(PIN_BUZ);
      buzzerOn = false;
    }

    buzzerChangedAt = now;
    return;
  }

  if (buzzerOn) {
    if (now - buzzerChangedAt >= BUZZER_ON_MS) {
      noTone(PIN_BUZ);
      buzzerOn = false;
      buzzerChangedAt = now;
    }
  } else {
    if (now - buzzerChangedAt >= BUZZER_OFF_MS) {
      tone(PIN_BUZ, BUZZER_FREQ);
      buzzerOn = true;
      buzzerChangedAt = now;
    }
  }
}

// --------------------------------------------------
// APPLY OUTPUTS
// --------------------------------------------------
void updateOutputs() {
  if (state == FAULT) {
    duty = 0.0f;

    analogWrite(PIN_PWM, 0);
    digitalWrite(PIN_EN, LOW);

    digitalWrite(PIN_FLT, HIGH);
    digitalWrite(PIN_LRUN, LOW);
    digitalWrite(PIN_LIDLE, LOW);
  } else {
    int pwmValue = (int)(duty * 255.0f / 100.0f);

    analogWrite(PIN_PWM, pwmValue);
    digitalWrite(PIN_EN, state == RUN);
    digitalWrite(PIN_FLT, LOW);
    digitalWrite(PIN_LRUN, state == RUN);
    digitalWrite(PIN_LIDLE, state == IDLE);
  }
}

// --------------------------------------------------
// SETUP
// --------------------------------------------------
void setup() {
  Serial.begin(115200);
  analogReadResolution(10);

  pinMode(PIN_START, INPUT_PULLUP);
  pinMode(PIN_STOP, INPUT_PULLUP);

  pinMode(PIN_PWM, OUTPUT);
  pinMode(PIN_EN, OUTPUT);
  pinMode(PIN_FLT, OUTPUT);
  pinMode(PIN_BUZ, OUTPUT);
  pinMode(PIN_LRUN, OUTPUT);
  pinMode(PIN_LIDLE, OUTPUT);

  analogWrite(PIN_PWM, 0);
  digitalWrite(PIN_EN, LOW);
  digitalWrite(PIN_FLT, LOW);
  digitalWrite(PIN_LRUN, LOW);
  digitalWrite(PIN_LIDLE, HIGH);
  noTone(PIN_BUZ);

  resetTimers();

  Serial.println("Motor Protection Controller ready");
}

// --------------------------------------------------
// MAIN LOOP
// --------------------------------------------------
void loop() {
  unsigned long now = millis();

  // Control and protection checks every 20 ms.
  if (now - lastCtl >= 20) {
    lastCtl = now;

    readSensors();

    // Detect faults in IDLE and RUN.
    if (state != FAULT) {
      Fault detected = checkFaults();

      if (detected != NONE) {
        enterFault(detected);
      }
    }

    switch (state) {
      case IDLE:
        duty = 0.0f;
        integ = 0.0f;
        target = 0.0f;

        if (!digitalRead(PIN_START) && state == IDLE) {
          Fault detected = checkFaults();

          if (detected != NONE) {
            enterFault(detected);
          } else {
            state = RUN;
            resetTimers();
            Serial.println("EVENT: START");
          }
        }
        break;

      case RUN:
        if (!digitalRead(PIN_STOP)) {
          state = IDLE;

          duty = 0.0f;
          integ = 0.0f;
          target = 0.0f;

          resetTimers();
          Serial.println("EVENT: STOP");
          break;
        }

        // Soft-start reference ramp.
        if (target < refRpm) {
          target += 30.0f;

          if (target > refRpm) {
            target = refRpm;
          }
        } else {
          target = refRpm;
        }

        // PI speed controller.
        {
          float error = target - rpm;

          integ = constrain(
            integ + error * 0.02f,
            -500.0f,
            500.0f
          );

          duty = constrain(
            0.02f * error + 0.05f * integ,
            0.0f,
            100.0f
          );
        }
        break;

      case FAULT:
        duty = 0.0f;
        integ = 0.0f;
        target = 0.0f;

        // Reset only when the fault condition is safe.
        if (!digitalRead(PIN_STOP) &&
            !faultStillActive()) {

          noTone(PIN_BUZ);
          buzzerOn = false;

          fault = NONE;
          state = IDLE;
          resetTimers();

          Serial.println("EVENT: fault cleared -> IDLE");
        }
        break;
    }

    updateOutputs();
  }

  // Run buzzer timing independently of the control loop.
  updateBuzzer();

  // Serial diagnostics every 5 seconds.
  if (now - lastPrint >= 5000) {
    lastPrint = now;

    Serial.print("RPM=");
    Serial.print((int)rpm);

    Serial.print(" I=");
    Serial.print(amps, 1);

    Serial.print("A T=");
    Serial.print((int)tempC);

    Serial.print("C REF=");
    Serial.print((int)refRpm);

    Serial.print(" PWM=");
    Serial.print((int)duty);

    Serial.print("% STATE=");
    Serial.print(stateName[state]);

    Serial.print(" FAULT=");
    Serial.println(faultName[fault]);
  }
}
