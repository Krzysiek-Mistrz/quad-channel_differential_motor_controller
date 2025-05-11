#include "motor_driver.h"

// <- 4 independent LEDC channels on ESP32
static const uint8_t ledcChannel[MOTOR_COUNT] = {0, 1, 2, 3};
static const uint32_t ledcFreq                = 5000;
static const uint8_t  ledcResolution          = 8;

void initMotorController() {
  for (int m = 0; m < MOTOR_COUNT; ++m) {
    uint8_t pwmPin = motorPins[m][0];
    uint8_t in1    = motorPins[m][1];
    uint8_t in2    = motorPins[m][2];
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    ledcSetup(ledcChannel[m], ledcFreq, ledcResolution);
    ledcAttachPin(pwmPin, ledcChannel[m]);
    ledcWrite(ledcChannel[m], 0);
  }
}

void setMotorSpeed(int m, int speed) {
  if (m < 0 || m >= MOTOR_COUNT) return;
  uint8_t in1 = motorPins[m][1];
  uint8_t in2 = motorPins[m][2];
  uint32_t maxDuty = (1 << ledcResolution) - 1;
  uint32_t duty = constrain(abs(speed), 0, maxDuty);
  if (speed > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (speed < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
  ledcWrite(ledcChannel[m], duty);
}

void setMotorSpeeds(const int speeds[MOTOR_COUNT]) {
  for (int i = 0; i < MOTOR_COUNT; ++i) {
    setMotorSpeed(i, speeds[i]);
  }
}
