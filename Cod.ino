#include <QTRSensors.h>
const int m11Pin = 7;
const int m12Pin = 6;
const int m21Pin = 5;
const int m22Pin = 4;
const int m1Enable = 11;
const int m2Enable = 10;
 
int m1Speed = 0;
int m2Speed = 0;
int motorSpeed = 0;
 
float kp = 4;
float ki = 0.0001;
float kd = 15;
 
float p = 1;
float i = 0;
float d = 0;
 
float error = 0;
float lastError = 0;
 
const int maxSpeed = 255;
const int minSpeed = -255;
 
const int baseSpeed = 255;
QTRSensors qtr;
 
long previousTime = 0;
long startCalibrationTime = 0;
int calibrateSpeed = 190;
int calibrationTotalTime = 3000;
int calibrationWaitTime = 200;
 
const int sensorCount = 6;
int sensorValues[sensorCount];
int sensors[sensorCount] = { 0, 0, 0, 0, 0, 0 };
int sensorThreshold = 350;
 
bool leftDirection = true;
bool white = false;
 
void setup() {
 
  // pinMode setup
  pinMode(m11Pin, OUTPUT);
  pinMode(m12Pin, OUTPUT);
  pinMode(m21Pin, OUTPUT);
  pinMode(m22Pin, OUTPUT);
  pinMode(m1Enable, OUTPUT);
  pinMode(m2Enable, OUTPUT);
 
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){ A0, A1, A2, A3, A4, A5 }, sensorCount);
 
  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
 
  startCalibrationTime = millis();
  Serial.begin(9600);
}
 
void calibrate() {
  qtr.calibrate();
  qtr.readLineBlack(sensorValues);
 
  white = true;
  for (int i = 0; i < sensorCount; i++) {
    if (sensorValues[i] > sensorThreshold)
      white = false;
  }
 
  if (millis() - previousTime >= calibrationWaitTime) {
    if (white) {
      previousTime = millis();
      leftDirection = !leftDirection;
    }
  }
 
  if (leftDirection)
    setMotorSpeed(calibrateSpeed, -calibrateSpeed);
  else
    setMotorSpeed(-calibrateSpeed, calibrateSpeed);
}
 
void loop() {
  if (millis() - startCalibrationTime <= calibrationTotalTime) {
    calibrate();
  } else {
    pidControl();
  }
}
 
void pidControl() {
  int error = map(qtr.readLineBlack(sensorValues), 0, 5000, -50, 50);
 
  p = error;
  i = i + error;
  d = error - lastError;
 
  motorSpeed = kp * p + ki * i + kd * d;
 
  m1Speed = baseSpeed;
  m2Speed = baseSpeed;
 
  if (error < 0) {
    m1Speed += motorSpeed;
  } else if (error > 0) {
    m2Speed -= motorSpeed;
  }
 
  m1Speed = constrain(m1Speed, minSpeed, maxSpeed);
  m2Speed = constrain(m2Speed, minSpeed, maxSpeed);
 
  setMotorSpeed(m1Speed, m2Speed);
}
 
// each arguments takes values between -255 and 255. The negative values represent the motor speed in reverse.
void setMotorSpeed(int motor1Speed, int motor2Speed) {
  if (motor1Speed == 0) {
    digitalWrite(m11Pin, LOW);
    digitalWrite(m12Pin, LOW);
    analogWrite(m1Enable, motor1Speed);
  } else {
    if (motor1Speed > 0) {
      digitalWrite(m11Pin, HIGH);
      digitalWrite(m12Pin, LOW);
      analogWrite(m1Enable, motor1Speed);
    }
    if (motor1Speed < 0) {
      digitalWrite(m11Pin, LOW);
      digitalWrite(m12Pin, HIGH);
      analogWrite(m1Enable, -motor1Speed);
    }
  }
  if (motor2Speed == 0) {
    digitalWrite(m21Pin, LOW);
    digitalWrite(m22Pin, LOW);
    analogWrite(m2Enable, motor2Speed);
  } else {
    if (motor2Speed > 0) {
      digitalWrite(m21Pin, HIGH);
      digitalWrite(m22Pin, LOW);
      analogWrite(m2Enable, motor2Speed);
    }
    if (motor2Speed < 0) {
      digitalWrite(m21Pin, LOW);
      digitalWrite(m22Pin, HIGH);
      analogWrite(m2Enable, -motor2Speed);
    }
  }
}
