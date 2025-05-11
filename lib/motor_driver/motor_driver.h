#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <Arduino.h>

#define MOTOR_COUNT 4

/* motorPins[motor][0]=PWM pin, [1]=IN1, [2]=IN2. Curr there are 4 motors
Motor 1 (left side)
Motor 2 (right side)
Motor 3 (right side)
Motor 4 (left side)
*/
static const uint8_t motorPins[MOTOR_COUNT][3] = {
  { 25, 18, 21 },
  { 26, 19, 22 },
  { 27, 23, 12 },
  { 14, 13, 15 }
};

/**
 * @brief   Initialize motor direction pins and PWM channels.
 */
void initMotorController();

/**
 * @brief   Set speed for a single motor.
 * @param   m       Motor index [0..MOTOR_COUNT-1]
 * @param   speed   Signed PWM: -255..+255
 */
void setMotorSpeed(int m, int speed);

/**
 * @brief   Set speeds for all motors.
 */
void setMotorSpeeds(const int speeds[MOTOR_COUNT]);

// two‐motor support (for gods sake)
inline void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  int arr[MOTOR_COUNT] = {leftSpeed, rightSpeed, rightSpeed, leftSpeed};
  setMotorSpeeds(arr);
}

#endif // MOTOR_DRIVER_H
