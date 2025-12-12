// ===== Pin Map (WORKING on Uno) =====
const int ENA = 9;   // Left PWM
const int IN1 = 3;   // Left dir
const int IN2 = 4;

const int ENB = 10;  // Right PWM
const int IN3 = 5;   // Right dir
const int IN4 = 6;

const int BRAKE_PIN = 7;   // switch to GND, INPUT_PULLUP
const int ACCEL_PIN = A1;  // pot wiper
const int LDR_PIN   = A0;  // LDR analog

const int TRIG_PIN = 12;
const int ECHO_PIN = 11;

const int LED_PIN = 13;    // built-in LED

// ===== Tuning =====
const int STOP_CM  = 25;
const int CLEAR_CM = 35;
const int MIN_DRIVE_PWM = 60;
const int AVOID_PWM = 140;

// ===== Motion helpers =====
enum Dir { COAST=0, FWD=1, REV=2, BRAKE=3 };

void setMotor(int en, int a, int b, Dir d, int pwm) {
  pwm = constrain(pwm, 0, 255);
  switch (d) {
    case FWD:   digitalWrite(a, HIGH); digitalWrite(b, LOW);  break;
    case REV:   digitalWrite(a, LOW);  digitalWrite(b, HIGH); break;
    case COAST: digitalWrite(a, LOW);  digitalWrite(b, LOW);  break;
    case BRAKE: digitalWrite(a, HIGH); digitalWrite(b, HIGH); break;
  }
  analogWrite(en, pwm);
}

void setLeft(Dir d, int pwm)  { setMotor(ENA, IN1, IN2, d, pwm); }
void setRight(Dir d, int pwm) { setMotor(ENB, IN3, IN4, d, pwm); }

bool brakePressed() { return digitalRead(BRAKE_PIN) == LOW; }

long readUltrasonicCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long dur = pulseIn(ECHO_PIN, HIGH, 25000UL);
  if (dur == 0) return -1;
  return (long)(dur / 58UL);
}

// ===== Smooth speed control =====
int currentPWM = 0;
int targetPWM  = 0;

unsigned long lastRampMs = 0;
const unsigned long RAMP_DT_MS = 15;
const int RAMP_STEP = 5;

void rampUpdate() {
  unsigned long now = millis();
  if (now - lastRampMs < RAMP_DT_MS) return;
  lastRampMs = now;

  if (currentPWM < targetPWM) currentPWM = min(currentPWM + RAMP_STEP, targetPWM);
  else if (currentPWM > targetPWM) currentPWM = max(currentPWM - RAMP_STEP, targetPWM);
}

// ===== State machine =====
enum State { DRIVE_FWD, AVOID_SPIN };
State state = DRIVE_FWD;

unsigned long lastSenseMs = 0;
long distCm = -1;

// LED blink timing
unsigned long lastLedMs = 0;
bool ledOn = false;

void updateLed() {
  unsigned long now = millis();

  if (currentPWM == 0) {
    digitalWrite(LED_PIN, LOW);
    return;
  }

  if (state == DRIVE_FWD) {
    digitalWrite(LED_PIN, HIGH);
    return;
  }

  // AVOID_SPIN: blink
  if (now - lastLedMs >= 150) {
    lastLedMs = now;
    ledOn = !ledOn;
    digitalWrite(LED_PIN, ledOn ? HIGH : LOW);
  }
}

void setup() {
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  pinMode(BRAKE_PIN, INPUT_PULLUP);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(LED_PIN, OUTPUT);

  setLeft(COAST, 0);
  setRight(COAST, 0);
}

void loop() {
  unsigned long now = millis();

  // Sensor sampling (~16Hz)
  if (now - lastSenseMs >= 60) {
    lastSenseMs = now;
    distCm = readUltrasonicCm();
  }

  int accelRaw = analogRead(ACCEL_PIN);
  (void)analogRead(LDR_PIN); // read it (not used yet)

  int accelPWM = map(accelRaw, 0, 1023, 0, 255);
  if (accelPWM < MIN_DRIVE_PWM) accelPWM = 0;

  // Brake override
  if (brakePressed()) {
    targetPWM = 0;
    rampUpdate();

    if (currentPWM == 0) {
      setLeft(COAST, 0);
      setRight(COAST, 0);
    } else {
      setLeft(FWD, currentPWM);
      setRight(FWD, currentPWM);
    }

    updateLed();
    return;
  }

  // State transitions based on ultrasonic
  if (distCm != -1) {
    if (state == DRIVE_FWD && distCm <= STOP_CM) state = AVOID_SPIN;
    if (state == AVOID_SPIN && distCm >= CLEAR_CM) state = DRIVE_FWD;
  }

  // Outputs by state
  if (state == DRIVE_FWD) {
    targetPWM = accelPWM;
    rampUpdate();

    if (currentPWM == 0) {
      setLeft(COAST, 0);
      setRight(COAST, 0);
    } else {
      setLeft(FWD, currentPWM);
      setRight(FWD, currentPWM);
    }
  } else {
    targetPWM = AVOID_PWM;
    rampUpdate();
    setLeft(FWD, currentPWM);
    setRight(REV, currentPWM);
  }

  updateLed();
}
