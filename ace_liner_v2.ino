/*
  ACE Liner V2 - PD line follower, starter sketch (11-channel digital sensor bar)
  Math: docs/ace_liner_v2_math.md

  EDIT BEFORE UPLOAD: every pin and gain below is a placeholder.
  Avoid D13 if your board has the buzzer on it.
  Motor driver: TB6612FNG (tie STBY to 5V, no pin used).

  PIN COUNT WARNING: 11 sensors + 6 motor pins + the JSumo start module is more than a Nano
  has free. If your bar reads through a shift register, multiplexer, or expansion board,
  replace the body of readBar() and leave the rest as is.

  Logic:
    e = weighted position of the line (mm, negative = line on the left)
    S = number of sensors on the line
    S == 0         line lost: pivot toward the side it was last seen
    S >= S_INT     intersection: ignore e, drive straight for INTERSECTION_MS
    otherwise      PD on e
*/

// ---------- Pins (EDIT) ----------
const uint8_t N = 11;
const uint8_t SENSOR_PINS[N] = {2, 3, 4, 7, 8, 12, A0, A1, A2, A3, A4};   // left to right (EDIT)
const bool    LINE_IS_HIGH   = true;                   // true if the sensor reads HIGH on the line

const uint8_t PWMA = 5,  AIN1 = 6,  AIN2 = 9;          // left motor
const uint8_t PWMB = 10, BIN1 = 11, BIN2 = A5;         // right motor

// ---------- Geometry and gains (EDIT, then tune) ----------
const float SENSOR_SPACING_MM = 8.0;      // d
const uint8_t S_INT = 6;                  // sensors on at once that count as an intersection (see math page, section 4)
const int   BASE_SPEED   = 140;           // 0-255
const float KP = 4.0;                     // PWM per mm of error
const float KD = 0.4;                     // PWM per (mm/s). Raise it to damp the wiggle
const int   TURN_SPEED   = 120;           // pivot speed when the line is lost
const unsigned long INTERSECTION_MS = 120;  // about (L + crossing width) / v
const unsigned long LOOP_US = 2000;       // 500 Hz
const unsigned long START_DELAY_MS = 3000;

const bool LOG_SERIAL = true;             // print e and u for tuning (115200 baud)

// ---------- State ----------
float prevE = 0;
int   lastSign = 1;                       // side the line was last seen on
unsigned long lastLoop = 0, lastLog = 0;

void setMotor(uint8_t pwm, uint8_t in1, uint8_t in2, int speed) {
  speed = constrain(speed, -255, 255);
  digitalWrite(in1, speed >= 0 ? HIGH : LOW);
  digitalWrite(in2, speed >= 0 ? LOW : HIGH);
  analogWrite(pwm, abs(speed));
}

void drive(int left, int right) {
  setMotor(PWMA, AIN1, AIN2, left);
  setMotor(PWMB, BIN1, BIN2, right);
}

// Fills on[i] with true when sensor i sees the line. Replace this body if the bar
// is not read with one pin per sensor.
void readBar(bool *on) {
  for (uint8_t i = 0; i < N; i++)
    on[i] = (digitalRead(SENSOR_PINS[i]) == HIGH) == LINE_IS_HIGH;
}

void setup() {
  for (uint8_t i = 0; i < N; i++) pinMode(SENSOR_PINS[i], INPUT);
  pinMode(PWMA, OUTPUT); pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT); pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  Serial.begin(115200);
  delay(START_DELAY_MS);
  lastLoop = micros();
}

void loop() {
  if (micros() - lastLoop < LOOP_US) return;
  float dt = (micros() - lastLoop) / 1e6;
  lastLoop = micros();

  // read the sensor bar
  bool on[N];
  readBar(on);
  int S = 0;
  float sumX = 0;
  for (uint8_t i = 0; i < N; i++) {
    if (on[i]) {
      S++;
      sumX += (i - (N - 1) / 2.0) * SENSOR_SPACING_MM;
    }
  }

  if (S == 0) {                           // line lost: pivot toward where it was last seen
    drive(lastSign * TURN_SPEED, -lastSign * TURN_SPEED);
    return;
  }

  if (S >= S_INT) {                       // intersection: hold straight, ignore e
    drive(BASE_SPEED, BASE_SPEED);
    delay(INTERSECTION_MS);
    prevE = 0;
    return;
  }

  float e = sumX / S;                     // mm, positive = line on the right
  float de = (e - prevE) / dt;            // mm/s
  prevE = e;
  if (e > 0.5) lastSign = 1; else if (e < -0.5) lastSign = -1;

  float u = KP * e + KD * de;             // PWM difference
  int left  = BASE_SPEED + (int)u;        // line on the right -> left wheel faster -> turn right
  int right = BASE_SPEED - (int)u;
  drive(left, right);

  if (LOG_SERIAL && millis() - lastLog >= 10) {
    lastLog = millis();
    Serial.print(e); Serial.print(','); Serial.println(u);
  }
}
