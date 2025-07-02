#include <Deneyap_6EksenAtaletselOlcumBirimi.h>
#include <ESP32Servo.h>
#include <Wire.h>

#define MIN_PWM 1060
#define MAX_PWM 1940
#define NEUTRAL 1500

Servo motor[8];
LSM6DSM IMU;

// Gelen joystick sinyalleri (Python'dan)
int X1 = 1500, Y1 = 1500, X2 = 1500, Y2 = 1500;

// PID verileri
float actualRoll = 0.0, actualPitch = 0.0;
float rollSetpoint = 0.0, pitchSetpoint = 0.0;
float rollError = 0.0, pitchError = 0.0;
float rollIntegral = 0.0, pitchIntegral = 0.0;
float rollLastError = 0.0, pitchLastError = 0.0;
float Kp = 2.0, Ki = 0.01, Kd = 0.5;
float rollOut = 0.0, pitchOut = 0.0;

const float k45 = 0.7071;
float rollGain[8]  = {-k45, k45, k45, -k45, -k45, k45, k45, -k45};
float pitchGain[8] = {k45, k45, -k45, -k45, k45, k45, -k45, -k45};

unsigned long lastTime = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (!IMU.begin(0x6A)) {
    Serial.println("IMU baglanamadi");
    while (1);
  }
  Serial.println("IMU basarili");

  int motorPins[8] = {D0, D1, D2, D3, D4, D5, D6, D7};
  for (int i = 0; i < 8; i++) {
    motor[i].attach(motorPins[i], MIN_PWM, MAX_PWM);
    motor[i].writeMicroseconds(MIN_PWM);
  }

  lastTime = millis();
}

void loop() {
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0f;
  lastTime = now;

  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.startsWith("X1:")) {
      parseJoystick(input);
    }
  }

  float gyroX = IMU.readRawGyroX();
  float gyroY = IMU.readRawGyroY();

  actualPitch += gyroX * dt;
  actualRoll += gyroY * dt;

  calculatePID(dt);
  controlMotors();
  printTelemetry();
  delay(10);
}

void parseJoystick(String data) {
  int vals[4] = {1500, 1500, 1500, 1500};
  int idx = 0;
  for (int i = 0; i < 4; i++) {
    int colon = data.indexOf(':', idx);
    if (colon == -1) break;
    int comma = data.indexOf(',', colon);
    if (comma == -1) comma = data.length();
    vals[i] = data.substring(colon + 1, comma).toInt();
    idx = comma + 1;
  }
  X1 = constrain(vals[0], 1000, 2000);
  Y1 = constrain(vals[1], 1000, 2000);
  X2 = constrain(vals[2], 1000, 2000);
  Y2 = constrain(vals[3], 1000, 2000);
}

void calculatePID(float dt) {
  pitchError = pitchSetpoint - actualPitch;
  rollError  = rollSetpoint  - actualRoll;

  pitchIntegral = constrain(pitchIntegral + pitchError * dt, -100, 100);
  rollIntegral  = constrain(rollIntegral  + rollError * dt,  -100, 100);

  float pitchDeriv = (pitchError - pitchLastError) / dt;
  float rollDeriv  = (rollError  - rollLastError)  / dt;

  pitchOut = Kp * pitchError + Ki * pitchIntegral + Kd * pitchDeriv;
  rollOut  = Kp * rollError  + Ki * rollIntegral  + Kd * rollDeriv;

  pitchLastError = pitchError;
  rollLastError  = rollError;
}

void controlMotors() {
  for (int i = 0; i < 8; i++) {
    int pwm = NEUTRAL;
    pwm += pitchOut * pitchGain[i];
    pwm += rollOut * rollGain[i];

    switch (i) {
      case 0: pwm += -X1 - X2 + Y2 + Y1 - 6000; break;
      case 1: pwm +=  X1 + X2 + Y2 + Y1 - 6000; break;
      case 2: pwm += -X1 + X2 - Y2 + Y1 - 6000; break;
      case 3: pwm +=  X1 - X2 - Y2 + Y1 - 6000; break;
      case 4: pwm += -X1 - X2 + Y2 - Y1 - 6000; break;
      case 5: pwm +=  X1 + X2 + Y2 - Y1 - 6000; break;
      case 6: pwm += -X1 + X2 - Y2 - Y1 - 6000; break;
      case 7: pwm +=  X1 - X2 - Y2 - Y1 - 6000; break;
    }

    pwm = constrain(pwm, MIN_PWM, MAX_PWM);
    motor[i].writeMicroseconds(pwm);
  }
}

void printTelemetry() {
  Serial.print("ROLL:"); Serial.print(actualRoll, 2);
  Serial.print(",PITCH:"); Serial.print(actualPitch, 2);
  Serial.print(",X1:"); Serial.print(X1);
  Serial.print(",Y1:"); Serial.print(Y1);
  Serial.print(",X2:"); Serial.print(X2);
  Serial.print(",Y2:"); Serial.print(Y2);

  for (int i = 0; i < 8; i++) {
    Serial.print(",M"); Serial.print(i+1);
    Serial.print(":"); Serial.print(motor[i].readMicroseconds());
  }
  Serial.println();
}