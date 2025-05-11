#ifndef ENCODER_DRIVER_H
#define ENCODER_DRIVER_H

#include <Arduino.h>

#define ENC_COUNT 4

// GPIO pins for each encoder channel A,B. We have 4 encoders curr.
static const uint8_t encPins[ENC_COUNT][2] = {
  {32, 33},
  {34, 35},
  {5, 17},
  {16, 4}
};
// internal counters
extern volatile long encCount[ENC_COUNT];

/**
 * @brief  Call in setup() to wire up interrupts for all ENC_COUNT channels.
 */
void initEncoderDriver();

/**
 * @brief  Read the current count for encoder i.
 */
long readEncoder(int i);

/**
 * @brief  Reset a single encoder's count to zero.
 */
void resetEncoder(int i);

/**
 * @brief  Reset all encoders' counts to zero.
 */
void resetEncoders();

#endif // ENCODER_DRIVER_H

