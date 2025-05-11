#define USE_BASE

#define BAUDRATE        57600
#define MAX_PWM         255
#define CMD_LINE_SIZE   64

#include <Arduino.h>
#include <string.h>
#include <stdlib.h>
#include "commands.h"

#ifdef USE_BASE
  #include "encoder_driver.h"
  #include "motor_driver.h"
  #include "diff_controller.h"
#endif

char cmdLine[CMD_LINE_SIZE];
uint8_t lineIndex = 0;
unsigned long nextPID = 0;
unsigned long lastMotorCommand = 0;

void processCommand(char *line) {
  char *tokens[6] = { nullptr };
  uint8_t ntok = 0;
  tokens[ntok] = strtok(line, " ");
  while (tokens[ntok] && ntok < 5) {
    tokens[++ntok] = strtok(nullptr, " ");
  }
  if (ntok == 0 || tokens[0][0] == '\0') return;
  auto toInt = [&](uint8_t i)->long { return (i<=ntok) ? atol(tokens[i]) : 0; };
  switch (tokens[0][0]) {
    case GET_BAUDRATE:
      Serial.println(BAUDRATE);
      break;
    case ANALOG_READ:
      Serial.println(analogRead(toInt(1)));
      break;
    case DIGITAL_READ:
      Serial.println(digitalRead(toInt(1)));
      break;
    case ANALOG_WRITE:
      analogWrite(toInt(1), toInt(2));
      Serial.println("OK");
      break;
    case DIGITAL_WRITE:
      digitalWrite(toInt(1), toInt(2) ? HIGH : LOW);
      Serial.println("OK");
      break;
    case PIN_MODE:
      pinMode(toInt(1), toInt(2) ? OUTPUT : INPUT);
      Serial.println("OK");
      break;

#ifdef USE_BASE
    case READ_ENCODERS: {
      for (int i = 0; i < ENC_COUNT; ++i) {
        Serial.print(readEncoder(i));
        if (i < ENC_COUNT-1) Serial.print(' ');
      }
      Serial.println();
      break;
    }
    case RESET_ENCODERS:
      resetEncoders();
      resetPID();
      Serial.println("OK");
      break;
    case MOTOR_SPEEDS: {
      lastMotorCommand = millis();
      float v = (ntok >= 2) ? atof(tokens[1]) : 0.0f;
      float w = (ntok >= 3) ? atof(tokens[2]) : 0.0f;
      setDiffTarget(v, w);
      Serial.println("OK");
      break;
    }
    case MOTOR_RAW_PWM: {
      lastMotorCommand = millis();
      resetPID();
      int raw[MOTOR_COUNT];
      for (int i = 0; i < MOTOR_COUNT; ++i) {
        raw[i] = (int)toInt(1 + i);
      }
      setMotorSpeeds(raw);
      Serial.println("OK");
      break;
    }
    case UPDATE_PID: {
      int params[4] = {0,0,0,0};
      char *p = tokens[1], *tok;
      uint8_t i = 0;
      while ((tok = strtok_r(p, ":", &p)) && i < 4) {
        params[i++] = atoi(tok);
      }
      leftPID.Kp = params[0];
      leftPID.Kd = params[1];
      leftPID.Ki = params[2];
      leftPID.Ko = params[3];
      rightPID = leftPID;
      Serial.println("OK");
      break;
    }
#endif
    default:
      Serial.println("Invalid Command");
      break;
  }
}

void setup() {
  Serial.begin(BAUDRATE);
#ifdef USE_BASE
  initEncoderDriver();
  initMotorController();
  float dt        = 1.0f / PID_RATE;
  float wheel_b   = 0.30f;
  float wheel_r   = 0.05f;
  initDiffController(dt, wheel_b, wheel_r,
                      1.2f, 0.01f, 0.1f,
                      1.2f, 0.01f, 0.1f);
  //resetPID(); if you want to use open loop only UNCOMMENT
  nextPID = millis() + PID_INTERVAL;
  lastMotorCommand = millis();
#endif
}

void loop() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\r' || ch == '\n') {
      if (lineIndex > 0) {
        cmdLine[lineIndex] = '\0';
        processCommand(cmdLine);
        lineIndex = 0;
      }
    } else if (lineIndex < CMD_LINE_SIZE - 1) {
      cmdLine[lineIndex++] = ch;
    }
  }

#ifdef USE_BASE
  unsigned long now = millis();
  // PID update & speeds
  if (now >= nextPID) {
    updatePID();
    nextPID += PID_INTERVAL;
  }
  if (now - lastMotorCommand > AUTO_STOP_INTERVAL) {
    int zeros[MOTOR_COUNT] = {0};
    setMotorSpeeds(zeros);
    lastMotorCommand = now;
  }
#endif
}