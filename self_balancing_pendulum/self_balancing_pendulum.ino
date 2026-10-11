// --- PIN CONFIGURATION ---
const int ENB = 9;   const int IN3 = 7;   const int IN4 = 8;    // Pendulum
const int ENA = 12;  const int IN1 = 10;  const int IN2 = 11;   // Cart
const int SENSOR_PIN = A0;

// --- SETTINGS ---
const int SENSOR_RAW_MAX = 1023;
const float TARGET_ANGLE = 0.0;
const float CATCH_ZONE   = 45.0;
const int   SWING_PWM    = 170;

// --- TIMING ---
unsigned long startupTime = 0;
const unsigned long WIGGLE_DURATION = 2000; // 2 seconds of oscillations

// --- CART SETTINGS ---
int cartSpeeds[] = {100, 130, 150, 180, 220};
int currentRound = 0;
bool cartMovingForward = true;
unsigned long cartTimer = 0;
const unsigned long MOVE_DURATION = 2000; // 2 seconds per direction

// --- PID GAINS ---
float Kp = 18.0; float Kd = 1.4; float Ki = 0.6;

// --- STATE ---
float offset_angle = 0;
float last_error = 0, integral = 0, prev_angle = 0;
unsigned long last_time;

void setup() {
  Serial.begin(115200);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);

  Serial.println(">>> CALIBRATING... Keep pendulum at BOTTOM.");
  delay(2000);

  long sum = 0;
  for(int i = 0; i < 100; i++) { sum += analogRead(SENSOR_PIN); delay(5); }
  offset_angle = 180.0 - (((float)sum / 100.0 / SENSOR_RAW_MAX) * 360.0);

  startupTime = millis(); // Mark when the process starts
  cartTimer = millis();
  last_time = micros();
}

void loop() {
  unsigned long now_ms = millis();
  unsigned long now_us = micros();
  float dt = (now_us - last_time) / 1000000.0;
  if (dt < 0.005) return;
  last_time = now_us;

  // --- 1. CART LOGIC (Always Running) ---
  if (now_ms - cartTimer >= MOVE_DURATION) {
    cartTimer = now_ms;
    if (!cartMovingForward) {
      currentRound++;
      if (currentRound > 4) currentRound = 0;
    }
    cartMovingForward = !cartMovingForward;
    Serial.print("Cart Round: "); Serial.println(currentRound);
  }
  moveCart(cartSpeeds[currentRound], cartMovingForward);

  // --- 2. PENDULUM LOGIC ---
  float angle = getNormalizedAngle();
  float velocity = (angle - prev_angle) / dt;
  prev_angle = angle;
  float error = TARGET_ANGLE - angle;
  float motor_output = 0;

  // Check if we are still in the 2-second "Oscillation" window
  if (now_ms - startupTime < WIGGLE_DURATION) {
    // Phase 1: Wiggle to and fro every 250ms
    if ((now_ms / 250) % 2 == 0) motor_output = SWING_PWM;
    else motor_output = -SWING_PWM;

    if (now_ms % 500 == 0) Serial.println("[PENDULUM] Initial Oscillations...");
  }
  else {
    // Phase 2: Swing-Up + PID Balance
    if (abs(angle) <= CATCH_ZONE) {
      integral += error * dt;
      integral = constrain(integral, -120, 120);
      float derivative = (error - last_error) / dt;
      motor_output = (Kp * error) + (Ki * integral) + (Kd * derivative);
    } else {
      // Intelligent Swing-up
      if (velocity > 0.5) motor_output = SWING_PWM;
      else if (velocity < -0.5) motor_output = -SWING_PWM;
      else motor_output = (now_ms % 1000 < 500) ? SWING_PWM : -SWING_PWM;
      integral = 0;
    }
  }

  movePendulum(motor_output);
  last_error = error;
}

// --- Helper Functions ---

float getNormalizedAngle() {
  int raw = analogRead(SENSOR_PIN);
  float normalized = ((float)raw / SENSOR_RAW_MAX * 360.0) + offset_angle;
  while (normalized > 180)  normalized -= 360;
  while (normalized < -180) normalized += 360;
  return normalized;
}

void movePendulum(float output) {
  int pwm = constrain((int)abs(output), 0, 255);
  if (pwm > 0 && pwm < 45) pwm = 45;
  if (output > 0) { digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); }
  else if (output < 0) { digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); }
  else { digitalWrite(IN3, LOW); digitalWrite(IN4, LOW); pwm = 0; }
  analogWrite(ENB, pwm);
}

void moveCart(int speed, bool forward) {
  analogWrite(ENA, speed);
  if (forward) { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); }
  else { digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); }
}
