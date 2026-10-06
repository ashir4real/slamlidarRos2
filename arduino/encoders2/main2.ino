// ============================================================
// 4-Wheel Differential Drive with L298N + Encoder Odometry
// Receives: v:1.2 r:0.5\n   (linear m/s, angular rad/s)
// Sends:    x:1.23 y:0.45 theta:0.78\n
// ============================================================

// ---------- Pin definitions ----------
#define ENA_LEFT   5
#define IN1_LEFT   6
#define IN2_LEFT   7

#define ENB_RIGHT  10
#define IN3_RIGHT  9
#define IN4_RIGHT  8

#define ENC_LEFT_A  2
#define ENC_RIGHT_A 3

// ---------- Robot parameters (FINAL) ----------
const float WHEEL_RADIUS = 0.032;          // metres
const float TRACK_WIDTH = 0.25;            // metres
const float MAX_RPM = 90;
const float MAX_SPEED = (MAX_RPM / 60.0) * 2 * PI * WHEEL_RADIUS;
const float ENCODER_TICKS_PER_REV = 620;   // calibrated

// ---------- Serial handling ----------
String inputString = "";
unsigned long lastCommandTime = 0;
const unsigned long COMMAND_TIMEOUT = 500;

float target_linear = 0.0;
float target_angular = 0.0;

// ---------- Motor state ----------
int leftPWM = 0, rightPWM = 0;
volatile bool leftDir = HIGH;
volatile bool rightDir = HIGH;

// ---------- Encoders ----------
volatile long leftCount = 0, rightCount = 0;

// ---------- Odometry ----------
float x = 0.0, y = 0.0, theta = 0.0;
long lastLeftCount = 0, lastRightCount = 0;
unsigned long lastOdomMillis = 0;
bool wasMoving = false;

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);

  pinMode(ENA_LEFT, OUTPUT);
  pinMode(IN1_LEFT, OUTPUT);
  pinMode(IN2_LEFT, OUTPUT);
  pinMode(ENB_RIGHT, OUTPUT);
  pinMode(IN3_RIGHT, OUTPUT);
  pinMode(IN4_RIGHT, OUTPUT);

  pinMode(ENC_LEFT_A, INPUT_PULLUP);
  pinMode(ENC_RIGHT_A, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_A), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_A), rightEncoderISR, CHANGE);

  stopMotors();
  Serial.println("Arduino ready (encoder odometry)");
}

// ---------- Main loop ----------
void loop() {
  // 1. Read serial commands
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      if (inputString.length() > 0) {
        parseCommand(inputString);
        lastCommandTime = millis();
      }
      inputString = "";
    } else {
      if (c != '\r') inputString += c;
    }
  }

  // 2. Timeout – stop motors
  if (millis() - lastCommandTime > COMMAND_TIMEOUT) {
    target_linear = 0.0;
    target_angular = 0.0;
  }

  // 3. Update motor speeds
  computeMotorSpeeds(target_linear, target_angular);
  setMotors(leftPWM, leftDir, rightPWM, rightDir);

  // 4. Always update odometry from encoders
  updateOdometryFromEncoders();

  // 5. Send odometry at 20 Hz
  if (millis() - lastOdomMillis >= 50) {
    lastOdomMillis = millis();
    sendOdometry();
  }
}

// ---------- Command parser ----------
void parseCommand(String cmd) {
  cmd.trim();
  int vIdx = cmd.indexOf("v:");
  int rIdx = cmd.indexOf("r:");
  if (vIdx == -1 || rIdx == -1) return;
  int vStart = vIdx + 2;
  int vEnd = rIdx - 1;
  if (vEnd < vStart) vEnd = vStart;
  String vStr = cmd.substring(vStart, vEnd);
  vStr.trim();
  int rStart = rIdx + 2;
  String rStr = cmd.substring(rStart);
  rStr.trim();

  float v = vStr.toFloat();
  float r = rStr.toFloat();
  if (abs(v) < 0.01) v = 0.0;
  if (abs(r) < 0.01) r = 0.0;
  target_linear = v;
  target_angular = r;
}

// ---------- Motor control ----------
void computeMotorSpeeds(float linear, float angular) {
  float leftSpeed  = linear - (angular * TRACK_WIDTH / 2.0);
  float rightSpeed = linear + (angular * TRACK_WIDTH / 2.0);
  leftSpeed  = constrain(leftSpeed,  -MAX_SPEED, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, -MAX_SPEED, MAX_SPEED);

  int lp = (int)((leftSpeed  / MAX_SPEED) * 255);
  int rp = (int)((rightSpeed / MAX_SPEED) * 255);

  // Pivot boost: if we're pivoting, kick up the PWM
  if (abs(linear) < 0.02 && abs(angular) > 0.1) {
    // Commanded pivot in place — boost PWM
    if (lp > 0) lp = min(255, lp + 100);
    if (lp < 0) lp = max(-255, lp - 100);
    if (rp > 0) rp = min(255, rp + 100);
    if (rp < 0) rp = max(-255, rp - 100);
  }

  leftPWM  = abs(lp);
  leftDir  = (lp >= 0) ? HIGH : LOW;
  rightPWM = abs(rp);
  rightDir = (rp >= 0) ? HIGH : LOW;
}

void setMotors(int pwmL, bool dirL, int pwmR, bool dirR) {
  analogWrite(ENA_LEFT, pwmL);
  digitalWrite(IN1_LEFT, dirL ? HIGH : LOW);
  digitalWrite(IN2_LEFT, dirL ? LOW  : HIGH);

  analogWrite(ENB_RIGHT, pwmR);
  digitalWrite(IN3_RIGHT, dirR ? HIGH : LOW);
  digitalWrite(IN4_RIGHT, dirR ? LOW  : HIGH);
}

void stopMotors() {
  analogWrite(ENA_LEFT, 0);
  analogWrite(ENB_RIGHT, 0);
  digitalWrite(IN1_LEFT, LOW);
  digitalWrite(IN2_LEFT, LOW);
  digitalWrite(IN3_RIGHT, LOW);
  digitalWrite(IN4_RIGHT, LOW);
}

// ---------- Encoder ISRs ----------
void leftEncoderISR() {
  if (leftDir == HIGH) {
    leftCount++;
  } else {
    leftCount--;
  }
}

void rightEncoderISR() {
  if (rightDir == HIGH) {
    rightCount++;
  } else {
    rightCount--;
  }
}

// ---------- Odometry from encoders ----------
void updateOdometryFromEncoders() {
  // Read counts atomically
  noInterrupts();
  long lc = leftCount;
  long rc = rightCount;
  interrupts();

  long dL = lc - lastLeftCount;
  long dR = rc - lastRightCount;
  lastLeftCount = lc;
  lastRightCount = rc;

  if (dL == 0 && dR == 0) return;

  float distPerTick = (2.0 * PI * WHEEL_RADIUS) / ENCODER_TICKS_PER_REV;
  float leftDist  = dL * distPerTick;
  float rightDist = dR * distPerTick;

  float deltaDist  = (leftDist + rightDist) / 2.0;
  float deltaTheta = (rightDist - leftDist) / TRACK_WIDTH;

  if (abs(deltaTheta) < 1e-6) {
    x += deltaDist * cos(theta);
    y += deltaDist * sin(theta);
  } else {
    float radius = deltaDist / deltaTheta;
    theta += deltaTheta;
    x += radius * (sin(theta) - sin(theta - deltaTheta));
    y -= radius * (cos(theta) - cos(theta - deltaTheta));
  }

  theta = atan2(sin(theta), cos(theta));
}

void sendOdometry() {
  Serial.print("x:");
  Serial.print(x, 3);
  Serial.print(" y:");
  Serial.print(y, 3);
  Serial.print(" theta:");
  Serial.println(theta, 3);
}
