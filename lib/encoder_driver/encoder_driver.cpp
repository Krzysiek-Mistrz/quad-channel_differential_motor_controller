#include "encoder_driver.h"

// storage for each channel
volatile long encCount[ENC_COUNT] = {0};
// ISR handlers; increment on channel A rising edge
void IRAM_ATTR onEncoder0() { encCount[0]++; }
void IRAM_ATTR onEncoder1() { encCount[1]++; }
void IRAM_ATTR onEncoder2() { encCount[2]++; }
void IRAM_ATTR onEncoder3() { encCount[3]++; }

void initEncoderDriver() {
  for (int i = 0; i < ENC_COUNT; ++i) {
    pinMode(encPins[i][0], INPUT_PULLUP);
    pinMode(encPins[i][1], INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(encPins[i][0]),
                    (i == 0 ? onEncoder0 :
                     i == 1 ? onEncoder1 :
                     i == 2 ? onEncoder2 :
                              onEncoder3),
                    RISING);
  }
}

long readEncoder(int i) {
  if (i < 0 || i >= ENC_COUNT) return 0;
  noInterrupts();
  long val = encCount[i];
  interrupts();
  return val;
}

void resetEncoder(int i) {
  if (i < 0 || i >= ENC_COUNT) return;
  noInterrupts();
  encCount[i] = 0;
  interrupts();
}

void resetEncoders() {
  noInterrupts();
  for (int i = 0; i < ENC_COUNT; ++i) encCount[i] = 0;
  interrupts();
}
