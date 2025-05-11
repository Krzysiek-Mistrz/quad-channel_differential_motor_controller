#ifndef DIFF_CONTROLLER_H
#define DIFF_CONTROLLER_H

#include <Arduino.h>
#include "encoder_driver.h"
#include "motor_driver.h"

/* Define motor mappings for 4-motor differential drive
Motor 1 (left side)
Motor 4 (left side)
Motor 2 (right side)
Motor 3 (right side)
*/
#define LEFT_MOTOR_1   1
#define LEFT_MOTOR_2   4
#define RIGHT_MOTOR_1  2
#define RIGHT_MOTOR_2  3
#define PID_RATE      30
static const unsigned long PID_INTERVAL = 1000UL / PID_RATE;

/**
 * @brief PID data structure
 */
typedef struct {
  float Kp, Ki, Kd, Ko;
  long  TargetTicksPerFrame;
  long  Encoder;
  long  PrevEnc;
  long  ITerm;
  float output;
  long  PrevInput;
} PIDController;

// Two sides of differential drive
extern PIDController leftPID;
extern PIDController rightPID;
// Physical parameters
extern float wheel_radius;
extern float wheel_base;
extern float control_dt;
extern unsigned char moving;

/**
 * @brief Initialize diff-controller internals.
 * @param dt         Control loop period [s]
 * @param wheel_base Distance between left/right wheels [m]
 * @param wheel_rad  Wheel radius [m]
 * @param kp_left    Proportional gain for left side
 * @param ki_left    Integral gain for left side
 * @param kd_left    Derivative gain for left side
 * @param kp_right   Proportional gain for right side
 * @param ki_right   Integral gain for right side
 * @param kd_right   Derivative gain for right side
 */
void initDiffController(float dt,
                        float wheel_base_m,
                        float wheel_rad_m,
                        float kp_left, float ki_left, float kd_left,
                        float kp_right, float ki_right, float kd_right);

/**
 * @brief  Reset PID state (call after init or new command)
 */
void resetPID();

/**
 * @brief  Enqueue a new (v, ω) set-point for the differential controller.
 * @param  v   Forward velocity [m/s]
 * @param  w   Angular velocity [rad/s]
 */
void setDiffTarget(float v, float w);

/**
 * @brief  Run one PID update and drive motors
 */
void updatePID();

#endif // DIFF_CONTROLLER_H